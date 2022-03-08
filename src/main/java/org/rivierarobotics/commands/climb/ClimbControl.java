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

package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.robot.ControlMap;
import org.rivierarobotics.subsystems.climb.Climb;

public class ClimbControl extends CommandBase {
    private final Climb climb;
    private final Joystick leftJoystick;

    public ClimbControl() {
        this.climb = Climb.getInstance();
        this.leftJoystick = ControlMap.CO_DRIVER_LEFT;
        addRequirements(this.climb);
    }

    @Override
    public void initialize() {
        climb.setPiston(Climb.Position.LOW, true);
        climb.setPiston(Climb.Position.MID, true);
        climb.setPiston(Climb.Position.HIGH, true);
    }

    @Override
    public void execute() {
        var xSpeed = MathUtil.fitDeadband(-leftJoystick.getY()) * 11;
        this.climb.setVoltage(xSpeed);
    }
}

