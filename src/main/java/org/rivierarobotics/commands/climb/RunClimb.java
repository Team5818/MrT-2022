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

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class RunClimb extends SequentialCommandGroup {
    //TODO: Change Command to look like this:
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
        addCommands(
                new ClimbSetAngle(0),
                new ClimbSetPosition(Climb.ClimbModule.LOW),

                ParallelDeadlineGroup(),

                new SetPistonState(ClimbPistons.MID, true),
                new SetPistonState(ClimbPistons.HIGH, true),
                new ClimbSetPosition(ClimbLocation.MID),
                new SetPistonState(ClimbPistons.MID, false),
                new WaitCommand(1.5),
                new SetPistonState(ClimbPistons.LOW, true),
                new ClimbSetPosition(ClimbLocation.HIGH),
                new SetPistonState(ClimbPistons.HIGH, false),
                new WaitCommand(1.5),
                new SetPistonState(ClimbPistons.MID, true)
        );
    }
}
