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

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.ClimbPositions;

public class ClimbSetPositionSlow extends CommandBase {
    private final Climb climb;
    private final double target;

    public ClimbSetPositionSlow(ClimbPositions climbModule, boolean reversed) {
        this.climb = Climb.getInstance();
        this.target = climbModule.locationRadians * (reversed ? -1 : 1);
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setPosition(target);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(climb.getAngle(), target, 0.1);
    }
}
