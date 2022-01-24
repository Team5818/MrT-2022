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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.rivierarobotics.commands.drive.SetDriveVelocity;
import org.rivierarobotics.commands.drive.SetWheelbaseAngle;
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
                new OpenAllPistons(),
                //TODO: Change to use SetDriveAngle and not SetWheelbaseAngle. We want gyro == 0 not wheels == 0.
                new SetWheelbaseAngle(0),
                new ParallelDeadlineGroup(new WaitUntilCommand(Climb.ClimbModule.LOW.lSwitch::get), new SetDriveVelocity(1)),
                //TODO: Make sure to set drive vel to 0 once the switch is hit.
                new SetPistonState(Climb.ClimbModule.LOW,false),
                new WaitCommand(0.5),

                new ParallelDeadlineGroup(new WaitUntilCommand(Climb.ClimbModule.MID.lSwitch::get), new ClimbSetPosition(Climb.ClimbModule.MID)),
                new SetPistonState(Climb.ClimbModule.MID,false),
                new WaitCommand(0.5),
                new SetPistonState( Climb.ClimbModule.LOW, true),
                new WaitCommand(0.5),

                new ParallelDeadlineGroup(new WaitUntilCommand(Climb.ClimbModule.HIGH.lSwitch::get), new ClimbSetPosition(Climb.ClimbModule.HIGH)),
                new SetPistonState(Climb.ClimbModule.HIGH,false),
                new WaitCommand(0.5),
                new SetPistonState( Climb.ClimbModule.MID, true)
        );
    }
}
