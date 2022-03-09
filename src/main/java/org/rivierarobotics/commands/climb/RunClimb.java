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

package org.rivierarobotics.commands.climb;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.rivierarobotics.subsystems.climb.Climb;

public class RunClimb extends SequentialCommandGroup {
    private static final double voltage = 5;

    public RunClimb(boolean reversed) {
        final Climb.Position first;
        final Climb.Position last;
        final double modifier;
        if (reversed) {
            first = Climb.Position.HIGH;
            last = Climb.Position.LOW;
        } else {
            last = Climb.Position.HIGH;
            first = Climb.Position.LOW;
        }
        addCommands(
                //new SetDriveAngle(90, 0.2),
                new OpenAllPistons(),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(first)),
                        new ClimbSetPosition(Climb.Position.LOW, reversed)
                ),
                new TogglePiston(first, true, 0),
                new WaitCommand(0.2),
                //new SetDriveVelocity(0,0,0),
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(Climb.Position.MID)),
                        new InteruptableSetVoltage(reversed, voltage)),
                new InstantCommand(() -> Climb.getInstance().setVoltage(0)),
                new TogglePiston(Climb.Position.MID, true, 0),
//                new WaitCommand(0.2),
                new WaitPiston(Climb.Position.MID, 0.5, 1.5, reversed),
                new TogglePiston(first, false, 0),
//                new WaitCommand(0.3),
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(last)),
                        new InteruptableSetVoltage(reversed, voltage)),
                new InstantCommand(() -> Climb.getInstance().setVoltage(0)),
                new TogglePiston(last, true, 0),
                new WaitCommand(0.3),
                new WaitPiston(last, 0.5, 1.5, reversed),
                new TogglePiston(Climb.Position.MID, false, 2)
//                new ClimbSetPosition(Climb.Position.HIGH, reversed)
        );
    }
}