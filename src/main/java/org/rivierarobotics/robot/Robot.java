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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.auto.CrazyWildCollectionBouncyHousePath;
import org.rivierarobotics.commands.auto.Fast3Ball;
import org.rivierarobotics.commands.auto.FourBallLeftAuto;
import org.rivierarobotics.commands.auto.MLCenter;
import org.rivierarobotics.commands.auto.TwoBallLeftAuto;
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
import org.rivierarobotics.subsystems.shoot.ShooterConstant;
import org.rivierarobotics.subsystems.shoot.ShootingTables;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Limelight;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.aifield.FieldMesh;
import org.rivierarobotics.util.ml.MLCore;

import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class Robot extends TimedRobot {
    public static double autoStartTime = 0.0;
    private final Field2d field2d = new Field2d();
    private SendableChooser<Command> chooser;
    private boolean autoFlag = false;
    private boolean ran = false;

    @Override
    public void disabledPeriodic() {
        DriveTrain.getInstance().drive(0, 0, 0, true);
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void robotInit() {
        initializeAllSubsystems();
        initializeDefaultCommands();
        Gyro.getInstance().resetGyro();
        ShooterConstant.ACTUATOR_ZERO_TICKS = (float) FloppaActuator.getInstance().getTicks();

        var drive = Shuffleboard.getTab("Drive");
        drive.add(field2d)
                .withSize(6, 4)
                .withPosition(0, 0)
                .withWidget("Field");

        // Initialize custom loops
        addPeriodic(() -> DriveTrain.getInstance().periodicLogging(), 0.5, 0.0);
        addPeriodic(this::shuffleboardLogging, 0.5, 0.01);

        this.chooser = new SendableChooser<>();
        chooser.addOption("ML Center", new MLCenter());
        chooser.addOption("No Auto", null);
        chooser.addOption("2BallLeftML", new TwoBallLeftAuto());
        chooser.addOption("4BallLeftML", new FourBallLeftAuto());
        chooser.addOption("3BallRightML", new Fast3Ball());
        chooser.addOption("5ball", new CrazyWildCollectionBouncyHousePath());

        Shuffleboard.getTab("Autos").add(chooser);

        FieldMesh.getInstance();
        resetRobotPoseAndGyro();
        var threader = Executors.newSingleThreadScheduledExecutor();
        threader.scheduleWithFixedDelay(new Thread(() -> Gyro.getInstance().updateRotation2D()), 0, 5, TimeUnit.MILLISECONDS);
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);
    }

    @Override
    public void robotPeriodic() {
        DriveTrain.getInstance().updateSwerveStates();
        CommandScheduler.getInstance().run();
        var command = CommandScheduler.getInstance().requiring(IntakeRollers.getInstance());
        if (command == null && ran) {
            CommandScheduler.getInstance().schedule(
                new SetBeltVoltage(-CollectBalls.COLLECT_VOLTAGE)
                .andThen(new WaitCommand(0.1))
                .andThen(new SetBeltVoltage(0))
            );
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
        if (ControlMap.CO_DRIVER_BUTTONS.getRawButton(13)) {
            Logging.robotShuffleboard.getTab("Field")
                    .setEntry("Logging", false);
            return;
        }

        Logging.robotShuffleboard.getTab("Field")
                .setEntry("Logging", true)
                .setEntry("Robot Pose", DriveTrain.getInstance().getPoseEstimator().getRobotPose().toString());
        var sb = Logging.robotShuffleboard;

        var drive = sb.getTab("Drive");

        drive.setEntry("Turn P", DriveTrain.getInstance().getTurnSpeedP());
        drive.setEntry("Turn Min", DriveTrain.getInstance().getMinTurnSpeed());

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

        var trajectory = MLCore.getBallTrajectory(dt, Gyro.getInstance(), FieldMesh.getInstance());
        drive.setEntry("Valid Trajectory", trajectory != null);
        dt.periodicLogging();
        drive.setEntry("X Vel (m/s)", dt.getChassisSpeeds().vxMetersPerSecond);
        drive.setEntry("y Vel (m/s)", dt.getChassisSpeeds().vyMetersPerSecond);
        drive.setEntry("Turn Vel (deg/s)", Math.toDegrees(dt.getChassisSpeeds().omegaRadiansPerSecond));
        drive.setEntry("X Pose", dt.getPoseEstimator().getRobotPose().getX());
        drive.setEntry("Y Pose", dt.getPoseEstimator().getRobotPose().getY());
        drive.setEntry("Pose Angle", dt.getPoseEstimator().getRobotPose().getRotation().getDegrees());
        drive.setEntry("Robot Angle", dt.getPoseEstimator().getRobotPose().getRotation().getDegrees());
        drive.setEntry("Field Centric", dt.getFieldCentric());

        drive.setEntry("Gyro Angle", Gyro.getInstance().getRotation2d().getDegrees());
        drive.setEntry("Gyro Angle Raw", Gyro.getInstance().getRotation2d().getRadians());
        drive.setEntry("Target Rotation Angle", dt.getTargetRotationAngle());

        climb.setEntry("Climb Position", cl.getAngle());
        climb.setEntry("Climb Raw Ticks", cl.getRawAngle());
        climb.setEntry("Absolute ticks", cl.getDutyCyclePose());
        climb.setEntry("Switch Low", clc.isSwitchSet(ClimbPositions.LOW));
        climb.setEntry("Switch Mid", clc.isSwitchSet(ClimbPositions.MID));
        climb.setEntry("Switch High", clc.isSwitchSet(ClimbPositions.HIGH));
        climb.setEntry("Piston Low", clc.isPistonSet(ClimbPositions.LOW));
        climb.setEntry("Piston Mid", clc.isPistonSet(ClimbPositions.MID));
        climb.setEntry("Piston High", clc.isPistonSet(ClimbPositions.HIGH));
        climb.setEntry("kP", Climb.KP);
        climb.setEntry("Velocity", cl.getVelocity());

        limeLight.setEntry("Shooter Speed", floppShooter.getTargetVelocity());
        limeLight.setEntry("Distance", Limelight.getInstance().getDistance());

        var cc = CommandScheduler.getInstance().requiring(FloppaActuator.getInstance());
        if (cc != null) {
            limeLight.setEntry("CC Flop", cc.getName());
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

        shoot.setEntry("Flywheel Right V", floppShooter.getRightFlywheelSpeed());
        shoot.setEntry("Flywheel Left V", -floppShooter.getLeftFlywheelSpeed());
        shoot.setEntry("Target Speed", floppShooter.getTargetVelocity());
        shoot.setEntry("Actuator Angle", floppActuator.getAngle());
        shoot.setEntry("Actuator Tick Raw", floppActuator.getTicks());
        shoot.setEntry("Detected Red", intakeSensors.getColorSensor().getColor().red);
        shoot.setEntry("Detected Green", intakeSensors.getColorSensor().getColor().green);
        shoot.setEntry("Detected Blue", intakeSensors.getColorSensor().getColor().blue);
        shoot.setEntry("Ball Color", intakeSensors.getBallColor());
        shoot.setEntry("Is Alliance Ball", intakeSensors.isTeamBall());
        shoot.setEntry("Alliance Color", DriverStation.getAlliance().toString());

        limeLight.setEntry("LL Adjusted Dist", Limelight.getInstance().getAdjustedDistance(
            Limelight.getInstance().getDistance(), Limelight.getInstance().getTx()));
        limeLight.setEntry("LL Adjusted Angle", Limelight.getInstance().getAdjustedTxAndCalc());
        limeLight.setEntry("LL TX", Limelight.getInstance().getTx());
        limeLight.setEntry("Flop Tuning", floppShooter.getTargetVelocity());
        limeLight.setEntry("LL Assist Angle", Limelight.getInstance().getShootingAssistAngle());
        limeLight.setEntry("Correct Position", Limelight.getInstance().getLLAbsPose().toString());
        limeLight.setEntry("Target Angle", ShootingTables.createFloppaAngleTable()
            .getValue(Limelight.getInstance().getDistance()));
        limeLight.setEntry("Target Speed", ShootingTables.createFloppaSpeedTable()
            .getValue(Limelight.getInstance().getDistance()));
        limeLight.setEntry("Hood Angle", floppActuator.getAngle());

        field.setEntry("Drive Pos", dt.getPoseEstimator().getRobotPose().toString());
    }

    @Override
    public void autonomousInit() {
        this.autoFlag = true;
        initializeAllSubsystems();
        initializeDefaultCommands();

        resetRobotPoseAndGyro();
        Robot.autoStartTime = Timer.getFPGATimestamp();
        try {
            var command = chooser.getSelected();
            if (command != null) {
                CommandScheduler.getInstance().schedule(command);
            }
        } catch (Exception ignored) {
            // If this fails we need robot code to still try to work in teleop,
            // so unless debugging there is no case where we want this to throw anything.
        }
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopExit() {
        Climb.getInstance().killController();
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
        Climb.getInstance().resetZeros(false);
        Gyro.getInstance().resetGyro();
        DriveTrain.getInstance().getPoseEstimator().resetPose(new Pose2d(5, 7, Gyro.getInstance().getRotation2d()));
        DriveTrain.getInstance().drive(0, 0, 0, true);
    }

    private void initializeDefaultCommands() {
        CommandScheduler.getInstance().setDefaultCommand(DriveTrain.getInstance(), new SwerveControl());
        CommandScheduler.getInstance().setDefaultCommand(Climb.getInstance(), new ClimbControl());
        CommandScheduler.getInstance().setDefaultCommand(FloppaActuator.getInstance(), new ShooterControl());
    }
}

