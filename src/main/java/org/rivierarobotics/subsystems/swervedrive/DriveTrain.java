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
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.lib.shuffleboard.RSTab;
import org.rivierarobotics.lib.shuffleboard.RSTable;
import org.rivierarobotics.lib.shuffleboard.RSTileOptions;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.Gyro;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * Represents a swerve drive style drivetrain.
 */
public class DriveTrain extends SubsystemBase {
    private static DriveTrain swerveDriveTrain;

    public static DriveTrain getInstance() {
        if (swerveDriveTrain == null) {
            swerveDriveTrain = new DriveTrain();
        }
        return swerveDriveTrain;
    }

    public static final double MAX_SPEED = 1.0; // m/s
    public static final double MAX_CHANGE_IN_VELOCITY = 0.0005; //
    public static final double MAX_ANGULAR_SPEED = Math.PI * 1.4 / 3; // rad/s
    public static final double MAX_ANGULAR_ACCELERATION = Math.PI * 0.7 / 3; // rad/s
    public static final double STATE_SPACE_LOOP_TIME = 0.02; // s
    private static final double WHEEL_DIST_TO_CENTER = 0.254; //m
    private static final String[] DRIVE_IDS = new String[]{"FL", "FR", "BL", "BR"};

    private final Gyro gyro;
    private final SwerveModule[] swerveModules = new SwerveModule[4];
    private final Translation2d[] swervePosition = new Translation2d[4];
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final HolonomicDriveController holonomicDriveController;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private double startTime = Timer.getFPGATimestamp();
    private boolean isFieldCentric = true;
    private Trajectory trajectory = new Trajectory();
    private RSTable[] loggingTables = new RSTable[4];



    public double targetRotationAngle = 0;


    private final RSTab tab;

    private DriveTrain() {
        //Position relative to center of robot -> (0,0) is the center
        swervePosition[0] = new Translation2d(WHEEL_DIST_TO_CENTER, WHEEL_DIST_TO_CENTER); //FL
        swervePosition[1] = new Translation2d(WHEEL_DIST_TO_CENTER, -WHEEL_DIST_TO_CENTER); //FR
        swervePosition[2] = new Translation2d(-WHEEL_DIST_TO_CENTER, WHEEL_DIST_TO_CENTER); //BL
        swervePosition[3] = new Translation2d(-WHEEL_DIST_TO_CENTER, -WHEEL_DIST_TO_CENTER); //BR

        swerveModules[0] = new SwerveModule(MotorIDs.FRONT_LEFT_DRIVE, MotorIDs.FRONT_LEFT_STEER, -3984, false, true);
        swerveModules[1] = new SwerveModule(MotorIDs.FRONT_RIGHT_DRIVE, MotorIDs.FRONT_RIGHT_STEER, -2710 + 2048, false, true);
        swerveModules[2] = new SwerveModule(MotorIDs.BACK_LEFT_DRIVE, MotorIDs.BACK_LEFT_STEER, -1020, false, true);
        swerveModules[3] = new SwerveModule(MotorIDs.BACK_RIGHT_DRIVE, MotorIDs.BACK_RIGHT_STEER, -2891 + 2048, false, true);

        this.tab = Logging.robotShuffleboard.getTab("Swerve");
        this.gyro = Gyro.getInstance();
        this.swerveDriveKinematics = new SwerveDriveKinematics(
                swervePosition[0], swervePosition[1], swervePosition[2], swervePosition[3]
        );

        this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                gyro.getRotation2d(),
                new Pose2d(0, 0, gyro.getRotation2d()),
                swerveDriveKinematics,
                //Standard deviations of model states. Increase these numbers to trust your model's state estimates less.
                //This matrix is in the form [x, y, theta]^T, with units in meters and radians.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.2, 0.2, .01),
                // Standard deviations of the encoder and gyro measurements. Increase these numbers to trust sensor
                // readings from encoders and gyros less. This matrix is in the form [theta], with units in radians.
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01),
                //Standard deviations of the vision measurements. Increase these numbers to trust global measurements
                //from vision less. This matrix is in the form [x, y, theta]^T, with units in meters and radians.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01), //Vision Measurement stdev
                .02
        );

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

        var e = Executors.newSingleThreadScheduledExecutor();
        e.scheduleAtFixedRate(this::updateOdometry, 0,20,TimeUnit.MILLISECONDS);
    }

    public void setSwerveModuleAngle(double angle) {
        for (var m : swerveModules) {
            m.setSteeringMotorAngle(angle);
        }
    }

    public void setSwerveModuleVelocity(double vel) {
        for (var m : swerveModules) {
            m.setDriveMotorVelocity(vel);
        }
    }

    public void setFieldCentric(boolean fieldCentric){
        this.isFieldCentric = fieldCentric;
    }

    public boolean getFieldCentric(){
        return isFieldCentric;
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return this.swerveDriveKinematics;
    }

    private double[] limitSpeeds(double xSpeed, double ySpeed) {
        var cs = getChassisSpeeds();
        var currentSpeed = Math.sqrt(Math.pow(cs.vxMetersPerSecond, 2) + Math.pow(cs.vyMetersPerSecond, 2));
        var targetSpeed = Math.sqrt(Math.pow(xSpeed,2) + Math.pow(ySpeed,2));
        var speeds = new double[2];

        if (Math.abs(targetSpeed - currentSpeed) > MAX_CHANGE_IN_VELOCITY) {
            speeds[0] = cs.vxMetersPerSecond + (MAX_CHANGE_IN_VELOCITY /Math.abs(targetSpeed - currentSpeed)) * (xSpeed - cs.vxMetersPerSecond);
            speeds[1] = cs.vyMetersPerSecond + (MAX_CHANGE_IN_VELOCITY /Math.abs(targetSpeed - currentSpeed)) * (ySpeed - cs.vyMetersPerSecond);
        } else {
            speeds[0] = xSpeed;
            speeds[1] = ySpeed;
        }

        return speeds;
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
                        : new ChassisSpeeds(limitSpeeds(xSpeed, ySpeed)[0], limitSpeeds(xSpeed, ySpeed)[1], rot)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);
        for (int i = 0; i < swerveModuleStates.length; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public void drivePath(String path) {
        try {
            String trajectoryJSON = "paths/" + path + ".wpilib.json";
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);

            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            //TODO: Fix this by making a custom pose with angle = gyro angle
            //swerveDrivePoseEstimator.resetPosition(trajectory.getInitialPose(), gyro.getRotation2d());
            startTime = Timer.getFPGATimestamp();
        } catch (IOException exception) {
            throw new UncheckedIOException(exception);
        }
    }

    public void drivePath(Trajectory path) {
        trajectory = path;
        //TODO: Fix this by making a custom pose with angle = gyro angle
        //swerveDrivePoseEstimator.resetPosition(path.getInitialPose(), gyro.getRotation2d());
        startTime = Timer.getFPGATimestamp();
    }

    /**
     * Call this method periodically to follow a trajectory.
     * returns false when path is done.
     */
    public boolean followHolonomicController() {
        if (Timer.getFPGATimestamp() - startTime > trajectory.getTotalTimeSeconds()) {
            return false;
        }

        var state = trajectory.sample(Timer.getFPGATimestamp() - startTime);
        var controls = holonomicDriveController.calculate(
                swerveDrivePoseEstimator.getEstimatedPosition(),
                state,
                //It is possible to use custom angles here that do not correspond to pathweaver's rotation target
                new Rotation2d(0)
        );
        SmartDashboard.putNumber("Pose Rot", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("TARGET ROT", controls.omegaRadiansPerSecond);
        drive(controls.vxMetersPerSecond, controls.vyMetersPerSecond, controls.omegaRadiansPerSecond, true);
        return true;
    }

    public void updateOdometry() {
        swerveDrivePoseEstimator.update(
                gyro.getRotation2d(),
                swerveModules[0].getState(),
                swerveModules[1].getState(),
                swerveModules[2].getState(),
                swerveModules[3].getState()
        );
    }

    public Pose2d getRobotPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void resetPose() {
        swerveDrivePoseEstimator.resetPosition(new Pose2d(0, 0, new Rotation2d(0)), new Rotation2d(0));
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

    public void periodicLogging() {
        for (int i = 0; i < swerveModules.length; i++) {
            loggingTables[i].setEntry(DRIVE_IDS[i] + " Swerve Velocity", swerveModules[i].getVelocity());
            loggingTables[i].setEntry(DRIVE_IDS[i] + " Swerve Angle", swerveModules[i].getAbsoluteAngle());
            loggingTables[i].setEntry(DRIVE_IDS[i] + " Swerve Drive Voltage", swerveModules[i].getDriveVoltage());
            loggingTables[i].setEntry(DRIVE_IDS[i] + " Swerve Steer Voltage", swerveModules[i].getSteerVoltage());
            loggingTables[i].setEntry(DRIVE_IDS[i] + " Swerve Full Angle", Math.toDegrees(swerveModules[i].getAngle()));
            loggingTables[i].setEntry(DRIVE_IDS[i] + " Swerve Steer Velocity", swerveModules[i].getSteerMotorVel());
            loggingTables[i].setEntry(DRIVE_IDS[i] + " Swerve Target Rotation", swerveModules[i].getTargetRotation().getDegrees());
            loggingTables[i].setEntry(DRIVE_IDS[i] + " Swerve Abs Target Rotation", swerveModules[i].getTargetRotationClamped().getDegrees());
            loggingTables[i].setEntry(DRIVE_IDS[i] + " Swerve Target Velocity", swerveModules[i].getTargetVelocity());
            loggingTables[i].setEntry(DRIVE_IDS[i] + " Swerve Pos Ticks", swerveModules[i].getPosTicks());
            loggingTables[i].setEntry(DRIVE_IDS[i] + " Swerve Pos Ticks Drive", swerveModules[i].getDriveTicks());
        }
    }

    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    public void periodicStateSpaceControl() {
        for(var m : swerveModules) {
            m.followControllers();
        }
    }
}
