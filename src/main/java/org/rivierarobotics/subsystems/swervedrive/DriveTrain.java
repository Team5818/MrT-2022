/*
 * This file is part of MrT-2022, licensed under the GNU General Public License (GPLv3).
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

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.lib.shuffleboard.RSTab;
import org.rivierarobotics.lib.shuffleboard.RSTable;
import org.rivierarobotics.lib.shuffleboard.RSTileOptions;
import org.rivierarobotics.robot.ControlMap;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.swerve.PoseEstimator;

/**
 * Represents a swerve drive style drivetrain.
 */
public class DriveTrain extends SubsystemBase {
    private static DriveTrain INSTANCE;

    public static DriveTrain getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new DriveTrain();
        }
        return INSTANCE;
    }

    //Drive Speed Constants
    public static final double MAX_SPEED = 10; // m/s
    public static final double MAX_ACCELERATION = 0.5; // m/s
    public static final double MAX_ANGULAR_SPEED = Math.PI * 3 * 0.8; // rad/s
    public static final double MAX_ANGULAR_ACCELERATION = Math.PI * 3; // rad/s
    //Turn Constraints
    public static double MAX_TURN_SPEED = 10;
    public static double TOLERANCE = 1.5;
    //Module Mappings / Measurements
    private static final double WHEEL_DIST_TO_CENTER = 0.29; //m

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
    private double targetRotationAngle = 0.0;
    private double turnSpeedP = 0.05;
    private double minTurnSpeed = 0.42;

    private DriveTrain() {
        //Position relative to center of robot -> (0,0) is the center (m)
        swervePosition[0] = new Translation2d(WHEEL_DIST_TO_CENTER, WHEEL_DIST_TO_CENTER); // FL
        swervePosition[1] = new Translation2d(WHEEL_DIST_TO_CENTER, -WHEEL_DIST_TO_CENTER); // FR
        swervePosition[2] = new Translation2d(-WHEEL_DIST_TO_CENTER, WHEEL_DIST_TO_CENTER); // BL
        swervePosition[3] = new Translation2d(-WHEEL_DIST_TO_CENTER, -WHEEL_DIST_TO_CENTER); // BR

        swerveModules[0] = new SwerveModule(MotorIDs.FRONT_LEFT_DRIVE, MotorIDs.FRONT_LEFT_STEER, -2935 + 2048);
        swerveModules[1] = new SwerveModule(MotorIDs.FRONT_RIGHT_DRIVE, MotorIDs.FRONT_RIGHT_STEER, 3677 + 2048);
        swerveModules[2] = new SwerveModule(MotorIDs.BACK_LEFT_DRIVE, MotorIDs.BACK_LEFT_STEER, -2833 + 2048);
        swerveModules[3] = new SwerveModule(MotorIDs.BACK_RIGHT_DRIVE, MotorIDs.BACK_RIGHT_STEER, -3619 + 2048);

        this.tab = Logging.robotShuffleboard.getTab("Swerve");
        this.gyro = Gyro.getInstance();
        this.swerveDriveKinematics = new SwerveDriveKinematics(
                swervePosition[0], swervePosition[1], swervePosition[2], swervePosition[3]
        );

        this.swerveDrivePoseEstimator = new PoseEstimator(gyro, swerveDriveKinematics, swerveModules);

        this.holonomicDriveController = new HolonomicDriveController(
                //PID FOR X DISTANCE (kp of 1 = 1m/s extra velocity / m of error)
                new PIDController(1.2, 0.001, 0),
                //PID FOR Y DISTANCE (kp of 1.2 = 1.2m/s extra velocity / m of error)
                new PIDController(1.2, 0.001, 0),
                //PID FOR ROTATION (kp of 1 = 1rad/s extra velocity / rad of error)
                new ProfiledPIDController(0.1, 0.012, 0,
                        new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * 5, MAX_ANGULAR_ACCELERATION * 5))
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
        if (Math.abs(xSpeed) <= 0.05 && Math.abs(ySpeed) <= 0.05 && rot == 0) {
            for (int i = 0; i < swerveModuleStates.length; i++) {
                swerveModules[i].setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
            }
        } else {
            for (int i = 0; i < swerveModuleStates.length; i++) {
                swerveModules[i].setDesiredState(swerveModuleStates[i]);
            }
        }
    }

    public double getRotationSpeed() {
        double gyroAngle = MathUtil.wrapToCircle(gyro.getRotation2d().getDegrees());
        if (MathUtil.isWithinTolerance(gyroAngle, targetRotationAngle, TOLERANCE)) {
            return 0.0;
        }
        double targetAngle = MathUtil.wrapToCircle(targetRotationAngle);
        var diff = targetAngle - gyroAngle;
        if (Math.abs(diff) >= 180 && diff < 0) {
            diff += 360;
        }
        if (Math.abs(diff) >= 180 && diff > 0) {
            diff -= 360;
        }

        double vel = (turnSpeedP * (diff));
        return Math.signum(diff) * (Math.min(Math.abs(vel), MAX_ANGULAR_SPEED) + minTurnSpeed);
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

    public double getTurnSpeedP() {
        return turnSpeedP;
    }

    public void setTurnSpeedP(double turnSpeedP) {
        this.turnSpeedP = turnSpeedP;
    }

    public double getMinTurnSpeed() {
        return minTurnSpeed;
    }

    public void setMinTurnSpeed(double minTurnSpeed) {
        this.minTurnSpeed = minTurnSpeed;
    }

    public void periodicLogging() {
        if (DriverStation.isFMSAttached() || ControlMap.CO_DRIVER_BUTTONS.getRawButton(13)) {
            return;
        }
        for (int i = 0; i < swerveModules.length; i++) {
            loggingTables[i].setEntry("Swerve Velocity", swerveModules[i].getVelocity());
            loggingTables[i].setEntry("Swerve Angle", swerveModules[i].getAbsoluteAngle());
            loggingTables[i].setEntry("Swerve Full Angle", Math.toDegrees(swerveModules[i].getAngle()));
            loggingTables[i].setEntry("Swerve Pos Ticks", swerveModules[i].getPosTicks());
            loggingTables[i].setEntry("Swerve Pos Ticks Drive", swerveModules[i].getDriveTicks());
        }
    }

    public void updateSwerveStates() {
        for (var sm : swerveModules) {
            sm.updateSwerveInformation();
        }
    }
}
