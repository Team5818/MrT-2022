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
    private static final double voltage = 11;
    private Climb.Position first;
    private Climb.Position last;
    private double modifier;
    private boolean reversed;
    //
    //  Set Robot Angle to 0 degrees
    //  Set Climb to LOW
    //  parallel deadline command: {
    //   deadline: new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(Switch.Buttons.LOW))),
    //   set drive speed -Y amount
    //  }
    //  Close Low Piston
    //  wait 0.5 seconds
    //  parallel deadline command: {
    //      deadline: new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(Switch.Buttons.MID))),
    //      Set Climb to MID
    //  }
    //  Close Mid Piston
    //  wait (0.5)
    //  Open Low Piston
    //  parallel deadline command: {
    //      deadline: new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(Switch.Buttons.HIGH))),
    //      Set Climb to HIGH
    //  }
    //  Close HIGH Piston
    //  wait (0.5)
    //  Open MID Piston
    //  Set Climb to FINAL

    public RunClimb(boolean reversed) {
        super();
        if (reversed) {
            this.first = Climb.Position.HIGH;
            this.last = Climb.Position.LOW;
            this.modifier = 1;
            this.reversed = false;
        } else {
            this.last = Climb.Position.HIGH;
            this.first = Climb.Position.LOW;
            this.modifier = -1;
            this.reversed = true;
        }
        addCommands(
                new OpenAllPistons(),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(first)),
                        new ClimbSetPosition(Climb.Position.LOW, reversed)
                ),
                new SetPistonState(first, true, 0),
                new WaitCommand(0.2),
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(Climb.Position.MID)),
                        new InstantCommand(() -> Climb.getInstance().setVoltage(voltage * modifier))),
                new InstantCommand(() -> Climb.getInstance().setVoltage(0)),
                new SetPistonState(Climb.Position.MID, true, 0),
                new WaitCommand(0.2),
                new WaitPiston(Climb.Position.MID, 0.5, 1.5, reversed),
                new SetPistonState(first, false, 0),
                new WaitCommand(0.3),
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(last)),
                        new InstantCommand(() -> Climb.getInstance().setVoltage(voltage * modifier))),
                new InstantCommand(() -> Climb.getInstance().setVoltage(0)),
                new SetPistonState(last, true, 0),
                new WaitCommand(0.3),
                new WaitPiston(last, 1, 3, reversed),
                new SetPistonState(Climb.Position.MID, false, 0),
                new ClimbSetPosition(Climb.Position.HIGH, reversed)
        );
    }
}
