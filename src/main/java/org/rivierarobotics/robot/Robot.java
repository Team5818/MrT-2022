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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.commands.climb.ClimbControl;
import org.rivierarobotics.commands.drive.SwerveControl;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.aifield.AIFieldDisplay;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class Robot extends TimedRobot {
    private final Field2d field2d = new Field2d();

    @Override
    public void robotInit() {
        initializeAllSubsystems();
        initializeDefaultCommands();
        Logging.aiFieldDisplay = new AIFieldDisplay(20);
        Gyro.getInstance().resetGyro();

        var drive = Shuffleboard.getTab("Drive");
        drive.add(field2d)
                .withSize(6, 4)
                .withPosition(0, 0)
                .withWidget("Field");

        initializeCustomLoops();
    }

    boolean isOn = false;
    int tick = 0;
    boolean[] states = {false,true,false,true,false,true};

    @Override
    public void robotPeriodic() {
//        Logging.aiFieldDisplay.update();

        tick++;
        if(tick > 20) {
            boolean temp = states[5];
            System.arraycopy(states, 0, states, 1, 5);
            states[0] = temp;
            for(int i = 1; i <= 6; i++) {
                ControlMap.DRIVER_BUTTONS.setOutput(i, states[i - 1]);
            }
            isOn = !isOn;
            tick = 0;
        }

    }

    @Override
    public void teleopInit() {
        new ButtonConfiguration().initTeleop();
        initializeAllSubsystems();
        initializeDefaultCommands();
        DriveTrain.getInstance().resetPose();
        Gyro.getInstance().resetGyro();
    }

    private void shuffleboardLogging() {
        var sb = Logging.robotShuffleboard;
        try {
            SmartDashboard.putString("DT Command", CommandScheduler.getInstance().requiring(DriveTrain.getInstance()).getName());
        } catch (Exception e) {}
        var drive = sb.getTab("Drive");
        var climb = sb.getTab("Climb");
        var machineLearning = sb.getTab("ML");
        var dt = DriveTrain.getInstance();
        var cl = Climb.getInstance();
        var ml = MLCore.getInstance();

        MLObject ball = new MLObject("red", new BoundingBox(0,0,0,0), 1);

        try {
            ball = ml.getDetectedObjects().get("red").get(0);
        } catch (NullPointerException nullPointerException){ }

        machineLearning.setEntry("BallX", ball.relativeLocationX);
        machineLearning.setEntry("BallY", ball.relativeLocationY);
        machineLearning.setEntry("Ball Distance", ball.relativeLocationDistance);
        machineLearning.setEntry("BX", ball.tx);
        machineLearning.setEntry("BY", ball.ty);

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
        drive.setEntry("target rotation angle", dt.getTargetRotationAngle());

        climb.setEntry("Climb Ticks", cl.getAngle());
        climb.setEntry("Switch 1", cl.isSwitchSet(Climb.Position.LOW));
        climb.setEntry("Switch 2", cl.isSwitchSet(Climb.Position.MID));
        climb.setEntry("Switch 3", cl.isSwitchSet(Climb.Position.HIGH));
        climb.setEntry("Piston 1", cl.isPistonSet(Climb.Position.LOW));
        climb.setEntry("Piston 2", cl.isPistonSet(Climb.Position.MID));
        climb.setEntry("Piston 3", cl.isPistonSet(Climb.Position.HIGH));
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
        CommandScheduler.getInstance().setDefaultCommand(Climb.getInstance(), new ClimbControl());
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


