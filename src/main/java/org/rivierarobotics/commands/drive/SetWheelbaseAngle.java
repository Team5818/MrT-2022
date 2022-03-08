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

package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class SetWheelbaseAngle extends CommandBase {
    private final DriveTrain dt;
    private final double angle;

    public SetWheelbaseAngle(double angle) {
        this.angle = Math.toRadians(angle);
        this.dt = DriveTrain.getInstance();
        addRequirements(this.dt);
    }

    @Override
    public void initialize() {
        Logging.robotShuffleboard.getTab("Drive").setEntry("check", angle);
        dt.setSwerveModuleAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
