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

package org.rivierarobotics.commands.basic.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.climb.ClimbClaws;
import org.rivierarobotics.subsystems.climb.ClimbPositions;

public class SetPiston extends InstantCommand {
    private final ClimbClaws climbClaws;
    private final ClimbPositions climbPosition;
    private final boolean isEngaged;

    public SetPiston(ClimbPositions climbPosition, boolean isEngaged) {
        this.climbClaws = ClimbClaws.getInstance();
        this.isEngaged = isEngaged;
        this.climbPosition = climbPosition;
        addRequirements(climbClaws);
    }

    @Override
    public void initialize() {
        climbClaws.setPiston(climbPosition, isEngaged);
    }
}
