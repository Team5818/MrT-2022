/*
 * This file is part of Placeholder-2022, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.subsystems.swervedrive;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilterLatencyCompensator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.lib.shuffleboard.RSTab;
import org.rivierarobotics.lib.shuffleboard.RSTable;
import org.rivierarobotics.lib.shuffleboard.RSTileOptions;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.swerve.PoseEstimator;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Represents a swerve drive style drivetrain.
 */
public class DriveTrain extends SubsystemBase {

    public static DriveTrain getInstance() {
        if (swerveDriveTrain == null) {
            swerveDriveTrain = new DriveTrain();
        }
        return swerveDriveTrain;
    }

    private static DriveTrain swerveDriveTrain;


    //Drive Speed Constants
    public static final double MAX_SPEED = 10; // m/s
    public static final double MAX_ACCELERATION = 0.5; // m/s
    public static final double MAX_ANGULAR_SPEED = Math.PI * 3 * 0.8; // rad/s
    public static final double MAX_ANGULAR_ACCELERATION = Math.PI * 3; // rad/s
    //Module Mappings / Measurements
    private static final double WHEEL_DIST_TO_CENTER = 0.254; //m

    private final Gyro gyro;
    //Modules
    private final SwerveModule[] swerveModules = new SwerveModule[4];
    private final Translation2d[] swervePosition = new Translation2d[4];
    //Drive Controllers
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final HolonomicDriveController holonomicDriveController;
    //Pose Estimators
    private final PoseEstimator swerveDrivePoseEstimator;
    //Logging
    private final RSTable[] loggingTables = new RSTable[4];
    private final RSTab tab;
    //Flags
    private boolean isFieldCentric = true;
    private boolean useDriverAssist = false;
    public double targetRotationAngle = 0.0;


    private DriveTrain() {
        //Position relative to center of robot -> (0,0) is the center (m)
        swervePosition[0] = new Translation2d(WHEEL_DIST_TO_CENTER, WHEEL_DIST_TO_CENTER); //FL
        swervePosition[1] = new Translation2d(WHEEL_DIST_TO_CENTER, -WHEEL_DIST_TO_CENTER); //FR
        swervePosition[2] = new Translation2d(-WHEEL_DIST_TO_CENTER, WHEEL_DIST_TO_CENTER); //BL
        swervePosition[3] = new Translation2d(-WHEEL_DIST_TO_CENTER, -WHEEL_DIST_TO_CENTER); //BR

        swerveModules[0] = new SwerveModule(MotorIDs.FRONT_LEFT_DRIVE, MotorIDs.FRONT_LEFT_STEER, -2964 + 2048);
        swerveModules[1] = new SwerveModule(MotorIDs.FRONT_RIGHT_DRIVE, MotorIDs.FRONT_RIGHT_STEER, -1503 + 2048);
        swerveModules[2] = new SwerveModule(MotorIDs.BACK_LEFT_DRIVE, MotorIDs.BACK_LEFT_STEER, -622 + 2048);
        swerveModules[3] = new SwerveModule(MotorIDs.BACK_RIGHT_DRIVE, MotorIDs.BACK_RIGHT_STEER, -3775 + 2048);

        this.tab = Logging.robotShuffleboard.getTab("Swerve");
        this.gyro = Gyro.getInstance();
        this.swerveDriveKinematics = new SwerveDriveKinematics(
                swervePosition[0], swervePosition[1], swervePosition[2], swervePosition[3]
        );

        this.swerveDrivePoseEstimator = new PoseEstimator(gyro, swerveDriveKinematics, swerveModules);

        this.holonomicDriveController = new HolonomicDriveController(
                //PID FOR X DISTANCE (kp of 1 = 1m/s extra velocity / m of error)
                new PIDController(1, 0, 0),
                //PID FOR Y DISTANCE (kp of 1.2 = 1.2m/s extra velocity / m of error)
                new PIDController(1, 0, 0),
                //PID FOR ROTATION (kp of 1 = 1rad/s extra velocity / rad of error)
                new ProfiledPIDController(1, 0, 0,
                        new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION))
        );

        loggingTables[0] = new RSTable("FL", tab, new RSTileOptions(3, 4, 0, 0));
        loggingTables[1] = new RSTable("FR", tab, new RSTileOptions(3, 4, 3, 0));
        loggingTables[2] = new RSTable("BL", tab, new RSTileOptions(3, 4, 6, 0));
        loggingTables[3] = new RSTable("BR", tab, new RSTileOptions(3, 4, 9, 0));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);
        if (xSpeed <= 0.05 && ySpeed <= 0.05 && rot == 0) {
            for (int i = 0; i < swerveModuleStates.length; i++) {
                swerveModules[i].setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
            }
        } else {
            for (int i = 0; i < swerveModuleStates.length; i++) {
                swerveModules[i].setDesiredState(swerveModuleStates[i]);
            }
        }
    }

    public void setSwerveModuleAngle(double angle) {
        for (var m : swerveModules) {
            m.setDesiredState(new SwerveModuleState(0, new Rotation2d(angle)));
        }
    }

    public void setSwerveModuleVelocity(double vel) {
        for (var m : swerveModules) {
            m.setDriveMotorVelocity(vel);
        }
    }

    public HolonomicDriveController getHolonomicDriveController() {
        return holonomicDriveController;
    }

    public boolean useDriverAssist() {
        return useDriverAssist;
    }

    public void setUseDriverAssist(boolean useDriverAssist) {
        this.useDriverAssist = useDriverAssist;
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.isFieldCentric = fieldCentric;
    }

    public boolean getFieldCentric() {
        return isFieldCentric;
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return this.swerveDriveKinematics;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return swerveDriveKinematics.toChassisSpeeds(
                swerveModules[0].getState(),
                swerveModules[1].getState(),
                swerveModules[2].getState(),
                swerveModules[3].getState()
        );
    }

    public double getTargetRotationAngle() {
        return targetRotationAngle;
    }

    public void setTargetRotationAngle(double targetRotationAngle) {
        this.targetRotationAngle = targetRotationAngle;
    }

    public PoseEstimator getPoseEstimator() {
        return swerveDrivePoseEstimator;
    }

    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    public void periodicLogging() {
        if (DriverStation.isFMSAttached()) return;
        for (int i = 0; i < swerveModules.length; i++) {
            loggingTables[i].setEntry("Swerve Velocity", swerveModules[i].getVelocity());
            loggingTables[i].setEntry("Swerve Angle", swerveModules[i].getAbsoluteAngle());
            loggingTables[i].setEntry("Swerve Full Angle", Math.toDegrees(swerveModules[i].getAngle()));
            loggingTables[i].setEntry("Swerve Pos Ticks", swerveModules[i].getPosTicks());
            loggingTables[i].setEntry("Swerve Pos Ticks Drive", swerveModules[i].getDriveTicks());
        }
    }


}
