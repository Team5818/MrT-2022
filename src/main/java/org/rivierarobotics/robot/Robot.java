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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.commands.collect.CollectControl;
import org.rivierarobotics.commands.drive.SwerveControl;
import org.rivierarobotics.commands.shoot.ShooterControl;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Floppas;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class Robot extends TimedRobot {
    private final Field2d field2d = new Field2d();

    private int tick = 0;
    private int frame = 0;
    private boolean[][] states = {
            {false, false, true, true, false, false},
            {false, false, false, false, true, true},
            {true, true, false, false, false, false}
    };

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
    }

    @Override
    public void robotPeriodic() {
        Shuffleboard.update();
        var sb = Logging.robotShuffleboard;
        var limeLight = sb.getTab("LL");
        limeLight.setEntry("Hood Angle", Floppas.getInstance().getAngle());

        //Logging.aiFieldDisplay.update();

        //iterates through button frames
//        tick++;
//        if (tick > 20) {
//            for (int i = 1; i <= 6; i++) {
//                ControlMap.DRIVER_BUTTONS.setOutput(i, states[frame][i - 1]);
//            }
//            this.frame = frame >= states.length ? 0 : frame + 1;
//            this.tick = 0;
//        }
    }

    @Override
    public void teleopInit() {
        new ButtonConfiguration().initTeleop();
        initializeAllSubsystems();
        initializeDefaultCommands();
        Climb.getInstance().setOffset();
        Gyro.getInstance().resetGyro();
    }

    private void shuffleboardLogging() {
        var sb = Logging.robotShuffleboard;
        var drive = sb.getTab("Drive");
        var climb = sb.getTab("Climb");
        var collect = sb.getTab("collect");
        var ML = sb.getTab("ML");

        var dt = DriveTrain.getInstance();
        var cl = Climb.getInstance();
        var col = Intake.getInstance();
        var MLcore = MLCore.getInstance();
        field2d.setRobotPose(dt.getRobotPose());
        //DriveTrain.getInstance().periodicLogging();
        dt.periodicLogging();
        drive.setEntry("x vel (m/s)", dt.getChassisSpeeds().vxMetersPerSecond);
        drive.setEntry("y vel (m/s)", dt.getChassisSpeeds().vyMetersPerSecond);
        drive.setEntry("turn vel (deg/s)", Math.toDegrees(dt.getChassisSpeeds().omegaRadiansPerSecond));
        drive.setEntry("x pose", dt.getRobotPose().getX());
        drive.setEntry("y pose", dt.getRobotPose().getY());
        drive.setEntry("Robot Angle", dt.getRobotPose().getRotation().getDegrees());
        drive.setEntry("is field centric", dt.getFieldCentric());

        drive.setEntry("Gyro Angle", Gyro.getInstance().getRotation2d().getDegrees());
        drive.setEntry("Gyro Angle raw", Gyro.getInstance().getAngle());
        drive.setEntry("target rotation angle", dt.getTargetRotationAngle());
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

        BoundingBox defaultBallBox = new BoundingBox(0,0,0,0);

        MLObject ball = new MLObject("red", defaultBallBox, 1);

        try {
            ball = MLcore.getDetectedObjects().get("red").get(0);
        } catch (NullPointerException nullPointerException){
        }

        ML.setEntry("Red BallX", ball.relativeLocationY);
        ML.setEntry("Red BallY", ball.relativeLocationX);
        ML.setEntry("Red Ball Distance", ball.relativeLocationDistance);
        ML.setEntry("Red TX", ball.ty);
        ML.setEntry("Red TY", ball.tx);
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
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
        Climb.getInstance();
    }

    private void initializeDefaultCommands() {
        CommandScheduler.getInstance().setDefaultCommand(DriveTrain.getInstance(), new SwerveControl());
        //CommandScheduler.getInstance().setDefaultCommand(Climb.getInstance(), new ClimbControl());
<<<<<<< HEAD
        CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new CollectControl());
        CommandScheduler.getInstance().setDefaultCommand(Floppas.getInstance(), new ShooterControl());
=======
        //CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new CollectControl());
>>>>>>> ball descender implimented
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
    public void simulationPeriodic() {
        Logging.aiFieldDisplay.update();
    }
}

