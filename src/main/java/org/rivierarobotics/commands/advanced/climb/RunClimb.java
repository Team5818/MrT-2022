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


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.rivierarobotics.commands.basic.climb.ClimbSetAngle;
import org.rivierarobotics.commands.basic.climb.ClimbSetPosition;
import org.rivierarobotics.commands.basic.climb.ClimbSetPositionSlow;
import org.rivierarobotics.commands.basic.climb.ClimbSetVoltage;
import org.rivierarobotics.subsystems.climb.ClimbClaws;
import org.rivierarobotics.subsystems.climb.ClimbPositions;

public class RunClimb extends SequentialCommandGroup {
    private static final double RUN_VOLTAGE = 7.5;

    public RunClimb(boolean reversed) {
        final ClimbPositions first;
        final ClimbPositions last;
        if (reversed) {
            first = ClimbPositions.HIGH;
            last = ClimbPositions.LOW;
        } else {
            last = ClimbPositions.HIGH;
            first = ClimbPositions.LOW;
        }
        addCommands(
                new OpenAllPistons(),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> ClimbClaws.getInstance().isSwitchSet(first)),
                        new ClimbSetPosition(ClimbPositions.LOW, reversed)
                ),
                new TogglePiston(first, true, 0),
                new WaitCommand(0.25),
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> ClimbClaws.getInstance().isSwitchSet(ClimbPositions.MID)),
                        new InteruptableSetVoltage(reversed, RUN_VOLTAGE)),
                new ClimbSetVoltage(reversed, 0),
                new TogglePiston(ClimbPositions.MID, true, 0),
                new ClimbSetPositionSlow(ClimbPositions.MID, reversed).withTimeout(3),
                new TogglePiston(first, false, 0),
                new WaitCommand(0.15),
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> ClimbClaws.getInstance().isSwitchSet(last)),
                        new InteruptableSetVoltage(reversed, RUN_VOLTAGE * 1.15)),
                new ClimbSetVoltage(reversed, RUN_VOLTAGE * 0.5),
                new TogglePiston(last, true, 0),
                new ClimbSetVoltage(reversed, 0),
                new ClimbSetPositionSlow(ClimbPositions.HIGH, reversed).withTimeout(3),
                new TogglePiston(ClimbPositions.MID, false, 0),
                new ClimbSetAngle(-0.88, reversed),
                new WaitCommand(4)

        );
    }
}