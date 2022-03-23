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

package org.rivierarobotics.commands.advanced.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.climb.Climb;

public class InteruptableSetVoltage extends CommandBase {
    private final double modifier;
    private final double voltage;
    private final Climb climb;

    public InteruptableSetVoltage(boolean reversed, double voltage) {
        this.modifier = reversed ? 1 : -1;
        this.voltage = voltage;
        this.climb = Climb.getInstance();
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.setVoltage(climb.getPlay()? (voltage * modifier) : 0);
    }
}
