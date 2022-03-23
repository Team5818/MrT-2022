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

package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestCollectAuto extends SequentialCommandGroup {
    //TODO either uncomment these lines and fill in the actual comments or remove
    // if remove, I suggest deleting the class as it is unused.
    public TestCollectAuto() {
        addCommands(
                // deploy intake
                //new ParallelDeadlineGroup(new CollectToggle(false, true, true), new DrivePath("straight")).deadlineWith(new WaitCommand(5))
                // retract intake
        );
    }
}
