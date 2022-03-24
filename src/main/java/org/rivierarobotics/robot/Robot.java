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

package org.rivierarobotics.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.PathGeneration;
import org.rivierarobotics.commands.auto.DriveShoot;
import org.rivierarobotics.commands.auto.MLAuto;
import org.rivierarobotics.commands.basic.collect.SetBeltVoltage;
import org.rivierarobotics.commands.control.ClimbControl;
import org.rivierarobotics.commands.control.ShooterControl;
import org.rivierarobotics.commands.control.SwerveControl;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.ClimbClaws;
import org.rivierarobotics.subsystems.climb.ClimbPositions;
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

import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class Robot extends TimedRobot {
    private final Field2d field2d = new Field2d();
    private SendableChooser<Command> chooser;
    private boolean autoFlag = false;
    private boolean ran = false;

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

        this.chooser = new SendableChooser<>();
        chooser.addOption("Drive backwards", new PathGeneration(-2, 0));
        chooser.addOption("MLAUTO", new MLAuto());
        chooser.addOption("No Auto", null);
        chooser.addOption("SimpleShootR", new DriveShoot(true));
        chooser.addOption("SimpleShootL", new DriveShoot(false));
        chooser.setDefaultOption("Drive backwards", new PathGeneration(-2, 0));

        Shuffleboard.getTab("Autos").add(chooser);

        FieldMesh.getInstance();

        var threader = Executors.newSingleThreadScheduledExecutor();
        threader.scheduleWithFixedDelay(new Thread(() -> {
            Gyro.getInstance().updateRotation2D();
        }), 0, 5, TimeUnit.MILLISECONDS);
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);
    }

    @Override
    public void robotPeriodic() {
        DriveTrain.getInstance().updateSwerveStates();
        CommandScheduler.getInstance().run();
        var command = CommandScheduler.getInstance().requiring(IntakeRollers.getInstance());
        if (command == null && ran) {
            CommandScheduler.getInstance().schedule(new SetBeltVoltage(-CollectBalls.COLLECT_VOLTAGE).withTimeout(0.2));
            this.ran = false;
        }
        if (command != null) {
            this.ran = true;
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
        this.autoFlag = false;
    }

    private void shuffleboardLogging() {
//        Logging.robotShuffleboard.getTab("Field")
//                .setEntry("RPOSE", DriveTrain.getInstance().getPoseEstimator().getRobotPose().toString());
//        Logging.robotShuffleboard.getTab("Field")
//                .setEntry("RTA", Limelight.getInstance().getShootingAssistAngle());



        if (ControlMap.CO_DRIVER_BUTTONS.getRawButton(13)) {
            //TODO maybe also a shuffleboard boolean to show if logging is enabled or not
            return;
        }
        Logging.robotShuffleboard.getTab("Field")
                .setEntry("RPOSE", DriveTrain.getInstance().getPoseEstimator().getRobotPose().toString());
        var sb = Logging.robotShuffleboard;

        var drive = sb.getTab("Drive");

        drive.setEntry("Turn P", DriveTrain.getInstance().getTURN_SPEED_P());
        drive.setEntry("Turn Min", DriveTrain.getInstance().getMinTurnSpeed());
//        if (ControlMap.CO_DRIVER_BUTTONS.getRawButton(13)) {
//            return;
//        }

        var climb = sb.getTab("Climb");
        var collect = sb.getTab("collect");
        var ml = sb.getTab("ML");
        var limeLight = sb.getTab("LL");
        var shoot = sb.getTab("shoot");
        var field = sb.getTab("Field");

        var dt = DriveTrain.getInstance();
        var cl = Climb.getInstance();
        var clc = ClimbClaws.getInstance();
        var mlCore = MLCore.getInstance();
        var floppShooter = FloppaFlywheels.getInstance();
        var floppActuator = FloppaActuator.getInstance();
        var intakeSensors = IntakeSensors.getInstance();
        //TODO either remove comments or uncomment
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
        climb.setEntry("Switch low", clc.isSwitchSet(ClimbPositions.LOW));
        climb.setEntry("Switch mid", clc.isSwitchSet(ClimbPositions.MID));
        climb.setEntry("Switch high", clc.isSwitchSet(ClimbPositions.HIGH));
        climb.setEntry("Piston low", clc.isPistonSet(ClimbPositions.LOW));
        climb.setEntry("Piston mid", clc.isPistonSet(ClimbPositions.MID));
        climb.setEntry("Piston high", clc.isPistonSet(ClimbPositions.HIGH));
        climb.setEntry("Kp", Climb.KP);
        climb.setEntry("velocity", cl.getVelocity());

        limeLight.setEntry("shooter speed", floppShooter.getTargetVelocity());
        limeLight.setEntry("distance", Limelight.getInstance().getDistance());

        var cc = CommandScheduler.getInstance().requiring(FloppaActuator.getInstance());
        if (cc != null) {
            limeLight.setEntry("CC FLOP", cc.getName());
        }


        var redBalls = mlCore.getDetectedObjects().get("red");
        if (redBalls != null && redBalls.size() > 0) {
            var ball = redBalls.get(0);
            if (ball != null) {
                ml.setEntry("Red BallY", ball.relativeLocationY);
                ml.setEntry("Red BallX", ball.relativeLocationX);
                ml.setEntry("Red Ball Distance", ball.relativeLocationDistance);
                ml.setEntry("Red TX", ball.tx);
                ml.setEntry("Red TY", ball.ty);
            }
        }

        shoot.setEntry("flywheel right v", floppShooter.getRightFlywheelSpeed());
        shoot.setEntry("flywheel left v", -floppShooter.getLeftFlywheelSpeed());
        shoot.setEntry("target speed", floppShooter.getTargetVelocity());
        shoot.setEntry("actuator angle", floppActuator.getAngle());
        shoot.setEntry("actuator tick raw", floppActuator.getTicks());
        shoot.setEntry("Detected Red", intakeSensors.getColorSensor().getColor().red);
        shoot.setEntry("Detected Green", intakeSensors.getColorSensor().getColor().green);
        shoot.setEntry("Detected Blue", intakeSensors.getColorSensor().getColor().blue);
        shoot.setEntry("ball color", intakeSensors.getBallColor());
        shoot.setEntry("Is Alliance Ball", intakeSensors.isTeamBall());
        shoot.setEntry("alliance color", DriverStation.getAlliance().toString());

        limeLight.setEntry("LL Adjusted Dist", Limelight.getInstance().getAdjustedDistance());
        limeLight.setEntry("LL Adjusted Angle", Limelight.getInstance().getAdjustedTx());
        limeLight.setEntry("LL TX", Limelight.getInstance().getTx());
        limeLight.setEntry("flop tuning", floppShooter.getTargetVelocity());
        limeLight.setEntry("LL Assist Angle", Limelight.getInstance().getShootingAssistAngle());
        limeLight.setEntry("Correct Position", Limelight.getInstance().getLLAbsPose().toString());
        limeLight.setEntry("Target Ang", ShootingTables.getFloppaAngleTable().getValue(Limelight.getInstance().getDistance()));
        limeLight.setEntry("Target Speed", ShootingTables.getFloppaSpeedTable().getValue(Limelight.getInstance().getDistance()));


        limeLight = sb.getTab("LL");
        limeLight.setEntry("Hood Angle", floppActuator.getAngle());

        field.setEntry("drive pos", dt.getPoseEstimator().getRobotPose().toString());
    }

    @Override
    public void autonomousInit() {
        this.autoFlag = true;
        initializeAllSubsystems();
        initializeDefaultCommands();

        resetRobotPoseAndGyro();
        try {
            var command = chooser.getSelected();
            if (command != null) {
                CommandScheduler.getInstance().schedule(command);
            }
            //TODO not a great idea to be catching *all* Exceptions. only catch the ones you're looking for
        } catch (Exception ignored) {
            // Padding for checkstyle
        }
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
        DriveTrain.getInstance().getPoseEstimator().resetPose(new Pose2d(10, 10, Gyro.getInstance().getRotation2d()));
    }

    private void initializeDefaultCommands() {
        CommandScheduler.getInstance().setDefaultCommand(DriveTrain.getInstance(), new SwerveControl());
        CommandScheduler.getInstance().setDefaultCommand(Climb.getInstance(), new ClimbControl());
        CommandScheduler.getInstance().setDefaultCommand(FloppaActuator.getInstance(), new ShooterControl());
    }

    private void initializeCustomLoops() {
        //TODO either remove comments or uncomment
        // if removed, maybe no use for this method >> put addPeriodic in robotInit
        addPeriodic(() -> {
            DriveTrain.getInstance().periodicLogging();
        }, 0.5, 0.0);
        addPeriodic(this::shuffleboardLogging, 2.00, 0.01);
    }

    @Override
    public void simulationInit() {
        //TODO either remove comments or uncomment
        //DriveTrain.getInstance().resetPose(new Pose2d(20,20, new Rotation2d(50)));
    }
}

