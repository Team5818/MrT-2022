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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rivierarobotics.lib.shuffleboard.RobotShuffleboard;
import org.rivierarobotics.subsystems.swerveDrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.StateSpace.PositionStateSpaceModel;
import org.rivierarobotics.util.StateSpace.VelocityStateSpaceModel;

public class Robot extends TimedRobot {
    VelocityStateSpaceModel m;
    PositionStateSpaceModel p;
    public static final RobotShuffleboard shuffleboard = new RobotShuffleboard();

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logging.initialize();
        initializeAllSubsystems();
        Gyro.getInstance().resetGyro();

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        new ButtonConfiguration().initTeleop();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    private void initializeAllSubsystems() {
        DriveTrain.getInstance();
    }


}


