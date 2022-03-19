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

package org.rivierarobotics.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.PathGeneration;
import org.rivierarobotics.commands.auto.DriveShoot;
import org.rivierarobotics.commands.auto.MLAuto;
import org.rivierarobotics.commands.auto.OneBallSimpleAuto;
import org.rivierarobotics.commands.auto.ShootFender;
import org.rivierarobotics.commands.basic.collect.SetBeltVoltageWithTimeout;
import org.rivierarobotics.commands.control.ClimbControl;
import org.rivierarobotics.commands.control.ShooterControl;
import org.rivierarobotics.commands.control.SwerveControl;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.ClimbClaws;
import org.rivierarobotics.subsystems.intake.IntakeBelt;
import org.rivierarobotics.subsystems.intake.IntakePiston;
import org.rivierarobotics.subsystems.intake.IntakeRollers;
import org.rivierarobotics.subsystems.intake.IntakeSensors;
import org.rivierarobotics.subsystems.shoot.FloppaActuator;
import org.rivierarobotics.subsystems.shoot.FloppaFlywheels;
import org.rivierarobotics.subsystems.shoot.ShootingTables;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Limelight;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.aifield.FieldMesh;
import org.rivierarobotics.util.ml.MLCore;

import static org.rivierarobotics.subsystems.climb.ClimbPositions.HIGH;
import static org.rivierarobotics.subsystems.climb.ClimbPositions.LOW;
import static org.rivierarobotics.subsystems.climb.ClimbPositions.MID;

public class Robot extends TimedRobot {
    private final Field2d field2d = new Field2d();
    private boolean autoFlag = false;

    private SendableChooser<Command> chooser;


    @Override
    public void robotInit() {
        initializeAllSubsystems();
        initializeDefaultCommands();
        Gyro.getInstance().resetGyro();

        var drive = Shuffleboard.getTab("Drive");
        drive.add(field2d)
                .withSize(6, 4)
                .withPosition(0, 0)
                .withWidget("Field");

        initializeCustomLoops();

        chooser = new SendableChooser<>();
        chooser.addOption("Drive backwards", new PathGeneration(-2, 0));
        chooser.addOption("RShootAndCollect2", new OneBallSimpleAuto(true));
        chooser.addOption("LShootAndCollect2", new OneBallSimpleAuto(false));
        chooser.addOption("MLAUTO", new MLAuto());
        chooser.addOption("No Auto", null);
        chooser.addOption("SimpleShootR", new DriveShoot(true));
        chooser.addOption("SimpleShootL", new DriveShoot(false));
        chooser.setDefaultOption("Drive backwards", new PathGeneration(-2, 0));
        chooser.setDefaultOption("Fender", new ShootFender());

        Shuffleboard.getTab("Autos").add(chooser);

        FieldMesh.getInstance();
    }

    boolean ran = false;
    @Override
    public void robotPeriodic() {
        var command = CommandScheduler.getInstance().requiring(IntakeRollers.getInstance());
        if(command == null && ran) {
            CommandScheduler.getInstance().schedule(new SetBeltVoltageWithTimeout(-CollectBalls.COLLECT_VOLTAGE, 0.2));
            ran = false;
        }
        if(command != null) {
            ran = true;
        }
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        new ButtonConfiguration().initTeleop();
        initializeAllSubsystems();
        initializeDefaultCommands();

        if (!autoFlag) {
            resetRobotPoseAndGyro();
        }
        autoFlag = false;
    }

    private void shuffleboardLogging() {
        if (DriverStation.isFMSAttached() || true) return;
        var sb = Logging.robotShuffleboard;
        var drive = sb.getTab("Drive");
        var climb = sb.getTab("Climb");
        var collect = sb.getTab("collect");
        var ML = sb.getTab("ML");
        var limeLight = sb.getTab("LL");
        var shoot = sb.getTab("shoot");
        var field = sb.getTab("Field");

        var dt = DriveTrain.getInstance();
        var cl = Climb.getInstance();
        var clc = ClimbClaws.getInstance();
        var MLcore = MLCore.getInstance();
        var floppShooter = FloppaFlywheels.getInstance();
        var floppActuator = FloppaActuator.getInstance();
        //field2d.setRobotPose(dt.getPoseEstimator().getRobotPose());
        //DriveTrain.getInstance().periodicLogging();
        dt.periodicLogging();
        drive.setEntry("x vel (m/s)", dt.getChassisSpeeds().vxMetersPerSecond);
        drive.setEntry("y vel (m/s)", dt.getChassisSpeeds().vyMetersPerSecond);
        drive.setEntry("turn vel (deg/s)", Math.toDegrees(dt.getChassisSpeeds().omegaRadiansPerSecond));
        drive.setEntry("x pose", dt.getPoseEstimator().getRobotPose().getX());
        drive.setEntry("y pose", dt.getPoseEstimator().getRobotPose().getY());
        drive.setEntry("pose angle", dt.getPoseEstimator().getRobotPose().getRotation().getDegrees());

        drive.setEntry("Robot Angle", dt.getPoseEstimator().getRobotPose().getRotation().getDegrees());
        drive.setEntry("is field centric", dt.getFieldCentric());

        drive.setEntry("Gyro Angle", Gyro.getInstance().getRotation2d().getDegrees());
        drive.setEntry("Gyro Angle raw", Gyro.getInstance().getRotation2d().getRadians());
        drive.setEntry("target rotation angle", dt.getTargetRotationAngle());

        climb.setEntry("Climb Position", cl.getAngle());
        climb.setEntry("Switch low", clc.isSwitchSet(LOW));
        climb.setEntry("Switch mid", clc.isSwitchSet(MID));
        climb.setEntry("Switch high", clc.isSwitchSet(HIGH));
        climb.setEntry("Piston low", clc.isPistonSet(LOW));
        climb.setEntry("Piston mid", clc.isPistonSet(MID));
        climb.setEntry("Piston high", clc.isPistonSet(HIGH));
        climb.setEntry("Kp", cl.kp);
        climb.setEntry("velocity", cl.getVelocity());

        limeLight.setEntry("shooter speed", floppShooter.getTargetVelocity());
        limeLight.setEntry("distance", Limelight.getInstance().getDistance());

        var redBalls = MLcore.getDetectedObjects().get("red");
        if (redBalls != null && redBalls.size() > 0) {
            var ball = redBalls.get(0);
            if (ball != null) {
                ML.setEntry("Red BallY", ball.relativeLocationY);
                ML.setEntry("Red BallX", ball.relativeLocationX);
                ML.setEntry("Red Ball Distance", ball.relativeLocationDistance);
                ML.setEntry("Red TX", ball.tx);
                ML.setEntry("Red TY", ball.ty);
            }
        }

        shoot.setEntry("flywheel right v", floppShooter.getRightFlywheelSpeed());
        shoot.setEntry("flywheel left v", -floppShooter.getLeftFlywheelSpeed());
        shoot.setEntry("target speed", floppShooter.getTargetVelocity());
        shoot.setEntry("actuator angle", floppActuator.getAngle());
        shoot.setEntry("actuator tick raw", floppActuator.getTicks());

        limeLight.setEntry("LL Adjusted Dist", Limelight.getInstance().getAdjustedDistance());
        limeLight.setEntry("LL Adjusted Angle", Limelight.getInstance().getAdjustedTx());
        limeLight.setEntry("LL TX", Limelight.getInstance().getTx());
        limeLight.setEntry("flop tuning", floppShooter.getTargetVelocity());
        limeLight.setEntry("LL Assist Angle", Limelight.getInstance().getShootingAssistAngle());
        limeLight.setEntry("Correct Position", Limelight.getInstance().getLLAbsPose().toString());
        limeLight.setEntry("Target Ang", ShootingTables.getFloppaAngleTable().getValue(Limelight.getInstance().getDistance()));
        limeLight.setEntry("Target Speed", ShootingTables.getFloppaAngleTable().getValue(Limelight.getInstance().getDistance()));


        limeLight = sb.getTab("LL");
        limeLight.setEntry("Hood Angle", floppActuator.getAngle());

        field.setEntry("drive pos", dt.getPoseEstimator().getRobotPose().toString());
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autoFlag = true;
        initializeAllSubsystems();
        initializeDefaultCommands();

        resetRobotPoseAndGyro();
        try {
            var command = chooser.getSelected();
            if (command != null) {
                CommandScheduler.getInstance().schedule(command);
            }
        } catch (Exception ignored) {
        }
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    private void initializeAllSubsystems() {
        DriveTrain.getInstance();
        IntakePiston.getInstance();
        FloppaActuator.getInstance();
        FloppaFlywheels.getInstance();
        Limelight.getInstance();
        IntakeBelt.getInstance();
        IntakeRollers.getInstance();
        IntakeSensors.getInstance();
        Climb.getInstance();
        ClimbClaws.getInstance();
    }

    private void resetRobotPoseAndGyro() {
        Gyro.getInstance().resetGyro();
        DriveTrain.getInstance().getPoseEstimator().updateRobotPose(new Pose2d(10, 10, Gyro.getInstance().getRotation2d()));
    }

    private void initializeDefaultCommands() {
        CommandScheduler.getInstance().setDefaultCommand(DriveTrain.getInstance(), new SwerveControl());
        CommandScheduler.getInstance().setDefaultCommand(Climb.getInstance(), new ClimbControl());
        CommandScheduler.getInstance().setDefaultCommand(FloppaActuator.getInstance(), new ShooterControl());
    }

    private void initializeCustomLoops() {
        addPeriodic(() -> {
            DriveTrain.getInstance().periodicLogging();
        }, 0.5, 0.0);

        addPeriodic(this::shuffleboardLogging, 2.00, 0.01);

        //DO NOT REMOVE - DEPENDENCY OF THE SWERVES
        addPeriodic(() -> {
            DriveTrain.getInstance().updateSwerveStates();
        }, 0.02, 0.01);
    }

    @Override
    public void simulationInit() {
        //DriveTrain.getInstance().resetPose(new Pose2d(20,20, new Rotation2d(50)));
    }
}

