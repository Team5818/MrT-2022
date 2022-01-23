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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.commands.climb.ClimbControl;
import org.rivierarobotics.commands.drive.SwerveControl;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;

public class Robot extends TimedRobot {
    private final Field2d field2d = new Field2d();

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
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        shuffleboardLogging();
    }

    @Override
    public void teleopInit() {
        new ButtonConfiguration().initTeleop();
        DriveTrain.getInstance().resetPose();
    }

    private void shuffleboardLogging() {
        var sb = Logging.robotShuffleboard;

        var drive = sb.getTab("Drive");
        var climb = sb.getTab("Climb");
        var dt = DriveTrain.getInstance();
        var cl = Climb.getInstance();
        field2d.setRobotPose(dt.getRobotPose());
        drive.setEntry("x vel (m/s)", dt.getChassisSpeeds().vxMetersPerSecond);
        drive.setEntry("y vel (m/s)", dt.getChassisSpeeds().vyMetersPerSecond);
        drive.setEntry("turn vel (deg/s)", Math.toDegrees(dt.getChassisSpeeds().omegaRadiansPerSecond));
        drive.setEntry("x pose", dt.getRobotPose().getX());
        drive.setEntry("y pose", dt.getRobotPose().getY());
        drive.setEntry("Robot Angle", dt.getRobotPose().getRotation().getDegrees());
        drive.setEntry("Gyro Angle", Gyro.getInstance().getAngle());

        climb.setEntry("Climb Ticks", cl.getAngle());

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
}


