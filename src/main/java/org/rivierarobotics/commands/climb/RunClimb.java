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

import edu.wpi.first.wpilibj2.command.*;
import org.rivierarobotics.commands.drive.SetDriveAngle;
import org.rivierarobotics.commands.drive.SetDriveVelocity;
import org.rivierarobotics.subsystems.climb.Climb;

public class RunClimb extends SequentialCommandGroup {
    private static final double voltage = -9;

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

    public RunClimb() {
        super(
//                new SetDriveAngle(90, 0.2),
                new OpenAllPistons(),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(Climb.Position.LOW)),
                        new ClimbSetPosition(Climb.Position.LOW)
                ),
                new SetPistonState(Climb.Position.LOW,true, 0),
                new WaitCommand(0.3),
//                new SetDriveVelocity(0,0,0),
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(Climb.Position.MID)),
                        new InstantCommand(() -> Climb.getInstance().setVoltage(voltage))),
                new InstantCommand(() -> Climb.getInstance().setVoltage(0)),
                new SetPistonState(Climb.Position.MID,true, 0),
                new WaitCommand(0.3),
                new WaitPiston(Climb.Position.MID, 1, 2),
                new SetPistonState(Climb.Position.LOW, false, 0),
                new WaitCommand(0.3),
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> Climb.getInstance().isSwitchSet(Climb.Position.HIGH)),
                        new InstantCommand(() -> Climb.getInstance().setVoltage(voltage))),
                new InstantCommand(() -> Climb.getInstance().setVoltage(0)),
                new SetPistonState(Climb.Position.HIGH,true, 0),
                new WaitCommand(0.3),
                new WaitPiston(Climb.Position.HIGH, 1, 2),
                new SetPistonState(Climb.Position.MID, false, 0),
                new ClimbSetPosition(Climb.Position.HIGH)
       );
    }
}
