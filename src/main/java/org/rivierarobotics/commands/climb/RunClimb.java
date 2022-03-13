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
import org.rivierarobotics.commands.subsystems.climb.ClimbSetPosition;
import org.rivierarobotics.commands.subsystems.climb.OpenAllPistons;
import org.rivierarobotics.commands.subsystems.climb.TogglePiston;
import org.rivierarobotics.subsystems.climb.ClimbDepreciated;

public class RunClimb extends SequentialCommandGroup {
    private static final double voltage = 9;

    public RunClimb(boolean reversed) {
        final ClimbDepreciated.Position first;
        final ClimbDepreciated.Position last;
        final double modifier;
        if (reversed) {
            first = ClimbDepreciated.Position.HIGH;
            last = ClimbDepreciated.Position.LOW;
        } else {
            last = ClimbDepreciated.Position.HIGH;
            first = ClimbDepreciated.Position.LOW;
        }
        addCommands(
                //new SetDriveAngle(90, 0.2),
                new OpenAllPistons(),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> ClimbDepreciated.getInstance().isSwitchSet(first)),
                        new ClimbSetPosition(ClimbDepreciated.Position.LOW, reversed)
                ),
                new TogglePiston(first, true, 0),
                new WaitCommand(0.25),
                //new SetDriveVelocity(0,0,0),
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> ClimbDepreciated.getInstance().isSwitchSet(ClimbDepreciated.Position.MID)),
                        new InteruptableSetVoltage(reversed, voltage)),
                new InstantCommand(() -> ClimbDepreciated.getInstance().setVoltage(0)),
                new TogglePiston(ClimbDepreciated.Position.MID, true, 0),
//                new WaitCommand(0.2),
                new WaitPiston(ClimbDepreciated.Position.MID, 0.5, 1, reversed),
                new TogglePiston(first, false, 0),
                new WaitCommand(0.15),
//                new InteruptableSetVoltage(reversed, voltage).withTimeout(0.9),
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> ClimbDepreciated.getInstance().isSwitchSet(last)),
                        new InteruptableSetVoltage(reversed, voltage * 1.15)),
                new InstantCommand(() -> ClimbDepreciated.getInstance().setVoltage(voltage * 0.5)),
                new TogglePiston(last, true, 0),
//                new WaitCommand(0.3),
                new WaitPiston(last, 0.5, 1.0, reversed),
                new InstantCommand(() -> ClimbDepreciated.getInstance().setVoltage(0)),
                new TogglePiston(ClimbDepreciated.Position.MID, false, 0),
                new ClimbSetPosition(ClimbDepreciated.Position.HIGH, reversed)
        );
    }
}