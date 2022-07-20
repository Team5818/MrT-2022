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

package org.rivierarobotics.commands.advanced.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.climb.ClimbClaws;
import org.rivierarobotics.subsystems.climb.ClimbPositions;
import org.rivierarobotics.util.RRTimer;

public class RetryClicker extends CommandBase {
    private final ClimbClaws climbClaws;
    private final ClimbPositions positions;
    private RRTimer rrTimer;

    public RetryClicker(int counterMax, ClimbPositions positions) {
        this.climbClaws = ClimbClaws.getInstance();
        this.positions = positions;
        this.rrTimer = new RRTimer(counterMax * 50);
        addRequirements(climbClaws);
    }

    @Override
    public void initialize() {
        rrTimer.reset();
    }

    @Override
    public void execute() {
        if (!climbClaws.isSwitchSet(positions)) {
            this.rrTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return rrTimer.finished();
    }
}
