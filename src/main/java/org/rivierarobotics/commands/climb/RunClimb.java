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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.subsystems.climb.ClimbLocation;
import org.rivierarobotics.subsystems.climb.Pistons;

public class RunClimb extends SequentialCommandGroup {

    public RunClimb() {
        addCommands(
                new SetPistonState(Pistons.MID, true),
                new SetPistonState(Pistons.HIGH, true),
                new ClimbSetPosition(ClimbLocation.MID),
                new SetPistonState(Pistons.MID, false),
                new WaitCommand(1.5),
                new SetPistonState(Pistons.LOW, true),
                new ClimbSetPosition(ClimbLocation.HIGH),
                new SetPistonState(Pistons.HIGH, false),
                new WaitCommand(1.5),
                new SetPistonState(Pistons.MID, true)
        );
    }
}
