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
                //Open all pistons
                new OpenAllPistons(),
                //Begin moving until the switch is confirmed
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> ClimbClaws.getInstance().isSwitchSet(first)),
                        new ClimbSetPosition(ClimbPositions.LOW, reversed)
                ),
                //Engage the piston and wait to let the pneumatics catch up
                new TogglePiston(first, true, 0),
                new WaitCommand(0.25),
                //Begin moving until the switch is confirmed, and then stop
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> ClimbClaws.getInstance().isSwitchSet(ClimbPositions.MID)),
                        new InteruptableSetVoltage(reversed, RUN_VOLTAGE)),
                new ClimbSetVoltage(reversed, 0),
                //Engage the piston
                new TogglePiston(ClimbPositions.MID, true, 0),
                //Move to position where release of the first hook is possible
                new ClimbSetPositionSlow(ClimbPositions.MID, reversed).withTimeout(3),
                //Release, and give it a moment to do it properly
                new TogglePiston(first, false, 0),
                new WaitCommand(0.15),
                //Begin moving until the switch is set, then engage and continue moving to ensure grip
                new ParallelDeadlineGroup(new RetryClicker(15, last),
                        new InteruptableSetVoltage(reversed, RUN_VOLTAGE * 1.15)),
                new TogglePiston(last, true, 0),
                new InteruptableSetVoltage(reversed, RUN_VOLTAGE * 1.15).withTimeout(0.5),
                //Move to position where the second claw can be disengaged, and then do so
                new ClimbSetPositionSlow(ClimbPositions.HIGH, reversed).withTimeout(3),
                new TogglePiston(ClimbPositions.MID, false, 0),
                //Move to final rest position, wait is necessary for it to actually move because of some issue with the command ending
                new ClimbSetAngle(-0.88, reversed),
                new WaitCommand(4)
        );
    }
}