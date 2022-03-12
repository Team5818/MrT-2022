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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.commands.auto.*;
import org.rivierarobotics.commands.climb.ClimbControl;
import org.rivierarobotics.commands.collect.CollectControl;
import org.rivierarobotics.commands.drive.SwerveControl;
import org.rivierarobotics.commands.shoot.ShooterControl;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.lib.shuffleboard.RSTileOptions;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Floppas;
import org.rivierarobotics.subsystems.vision.Limelight;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.InterpolationTable;
import org.rivierarobotics.util.aifield.FieldMesh;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class Robot extends TimedRobot {
    private final Field2d field2d = new Field2d();
    private boolean autoFlag = false;

    private int tick = 0;
    private int frame = 0;
    private boolean[][] states = {
            {false, false, true, true, false, false},
            {false, false, false, false, true, true},
            {true, true, false, false, false, false}
    };
    private SendableChooser<Command> chooser;


    @Override
    public void robotInit() {
        initializeAllSubsystems();
        initializeDefaultCommands();
        DriveTrain.getInstance().resetPose();
        Gyro.getInstance().resetGyro();

        var drive = Shuffleboard.getTab("Drive");
        drive.add(field2d)
                .withSize(6, 4)
                .withPosition(0, 0)
                .withWidget("Field");

        initializeCustomLoops();

        Climb.getInstance().setOffset();

        chooser = new SendableChooser<>();
        chooser.addOption("Drive backwards", new PathGeneration(-2, 0));
        chooser.addOption("RShootAndCollect2", new OneBallSimpleAuto(true));
        chooser.addOption("LShootAndCollect2", new OneBallSimpleAuto(false));
        chooser.addOption("MLAUTO", new MLAuto());
        chooser.addOption("No Auto", null);
        chooser.addOption("SimpleShootR", new DriveShoot(true));
        chooser.addOption("SimpleShootL", new DriveShoot(false));
        chooser.setDefaultOption("Drive backwards", new PathGeneration(-2, 0));

        Shuffleboard.getTab("Autos").add(chooser);
    }

    @Override
    public void robotPeriodic() {
        var sb = Logging.robotShuffleboard;
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
        Climb.getInstance().setOffset();
    }

    private void shuffleboardLogging() {
        if (DriverStation.isFMSAttached()) return;
        var sb = Logging.robotShuffleboard;
        var drive = sb.getTab("Drive");
        var climb = sb.getTab("Climb");
        var collect = sb.getTab("collect");
        var ML = sb.getTab("ML");
        var limeLight = sb.getTab("LL");
        var shoot = sb.getTab("shoot");

        var dt = DriveTrain.getInstance();
        var cl = Climb.getInstance();
        var col = Intake.getInstance();
        var MLcore = MLCore.getInstance();
        var flopp = Floppas.getInstance();
        field2d.setRobotPose(dt.getRobotPose());
        //DriveTrain.getInstance().periodicLogging();
        dt.periodicLogging();
        drive.setEntry("x vel (m/s)", dt.getChassisSpeeds().vxMetersPerSecond);
        drive.setEntry("y vel (m/s)", dt.getChassisSpeeds().vyMetersPerSecond);
        drive.setEntry("turn vel (deg/s)", Math.toDegrees(dt.getChassisSpeeds().omegaRadiansPerSecond));
        drive.setEntry("x pose", dt.getRobotPose().getX());
        drive.setEntry("y pose", dt.getRobotPose().getY());
        drive.setEntry("pose angle", dt.getRobotPose().getRotation().getDegrees());

        drive.setEntry("Robot Angle", dt.getRobotPose().getRotation().getDegrees());
        drive.setEntry("is field centric", dt.getFieldCentric());
        drive.setEntry("minrot", SwerveControl.MIN_ROT);
        drive.setEntry("turnspeed", SwerveControl.TURN_SPEED);
        drive.setEntry("maxspeed", SwerveControl.MAX_SPEED);

        drive.setEntry("Gyro Angle", Gyro.getInstance().getRotation2d().getDegrees());
        drive.setEntry("Gyro Angle raw", Gyro.getInstance().getRotation2d().getRadians());
        drive.setEntry("target rotation angle", dt.getTargetRotationAngle());

        climb.setEntry("Climb Position", cl.getAngle());
        climb.setEntry("Climb Ticks", cl.getRawTicks());
        climb.setEntry("Switch low", cl.isSwitchSet(Climb.Position.LOW));
        climb.setEntry("Switch mid", cl.isSwitchSet(Climb.Position.MID));
        climb.setEntry("Switch high", cl.isSwitchSet(Climb.Position.HIGH));
        climb.setEntry("Piston low", cl.isPistonSet(Climb.Position.LOW));
        climb.setEntry("Piston mid", cl.isPistonSet(Climb.Position.MID));
        climb.setEntry("Piston high", cl.isPistonSet(Climb.Position.HIGH));

        collect.setEntry("ispositive", col.getIsPositive());
        collect.setEntry("belt voltage", col.getBeltVoltage());
        collect.setEntry("roller voltage", col.getIntakeVoltage());
        collect.setEntry("color sensor proximity", col.getColorSensorV3().getProximity());
        collect.setEntry("color sensor red", col.getColorSensorV3().getColor().red * 255);
        collect.setEntry("color sensor green", col.getColorSensorV3().getColor().green * 255);
        collect.setEntry("Color sensor Blue", col.getColorSensorV3().getColor().blue * 255);
        collect.setEntry("can collect", col.canCollect());
        collect.setEntry("color sensor occupied", col.colorSensorHasBall());
        collect.setEntry("proximity sensor has ball", col.distanceSensorHasBall());
        collect.setEntry("analogsensor", col.getDistanceSensor().getValue());
        collect.setEntry("Is Full", col.getIsFull());
        limeLight.setEntry("shooter speed", Floppas.getInstance().getTargetV());
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

        shoot.setEntry("flywheel right v", flopp.getRightSpeed());
        shoot.setEntry("flywheel left v", -flopp.getLeftSpeed());
        shoot.setEntry("actuator angle", flopp.getAngle());
        shoot.setEntry("right target", flopp.getTarget(false));
        shoot.setEntry("left target", flopp.getTarget(true));

        limeLight.setEntry("LL Adjusted Dist", Limelight.getInstance().getAdjustedDistance());
        limeLight.setEntry("LL Adjusted Angle", Limelight.getInstance().getAdjustedTx());
        limeLight.setEntry("LL TX", Limelight.getInstance().getTx());
        limeLight.setEntry("flop tuning", flopp.getTargetV());


        limeLight = sb.getTab("LL");
        limeLight.setEntry("Hood Angle", Floppas.getInstance().getAngle());
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

        Climb.getInstance().setOffset();
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
        Floppas.getInstance();
        Limelight.getInstance();
        Intake.getInstance();
        Climb.getInstance();
    }

    private void resetRobotPoseAndGyro() {
        DriveTrain.getInstance().updateRobotPose(new Pose2d(10, 10, Gyro.getInstance().getRotation2d()));
        Gyro.getInstance().resetGyro();
    }

    private void initializeDefaultCommands() {
        CommandScheduler.getInstance().setDefaultCommand(DriveTrain.getInstance(), new SwerveControl());
        CommandScheduler.getInstance().setDefaultCommand(Climb.getInstance(), new ClimbControl());
//        CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new CollectControl());
        CommandScheduler.getInstance().setDefaultCommand(Floppas.getInstance(), new ShooterControl());
    }

    private void initializeCustomLoops() {
        addPeriodic(() -> {
            DriveTrain.getInstance().periodicStateSpaceControl();
        }, DriveTrain.STATE_SPACE_LOOP_TIME, 0.0);
        addPeriodic(() -> {
            DriveTrain.getInstance().periodicLogging();
        }, 0.5, 0.0);
        addPeriodic(this::shuffleboardLogging, 0.5, 0.0);
    }

    @Override
    public void simulationInit() {
        //DriveTrain.getInstance().resetPose(new Pose2d(20,20, new Rotation2d(50)));
    }
}

