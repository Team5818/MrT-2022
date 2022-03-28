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

package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.DrivePathPlannerPath;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.basic.collect.SetIntakeState;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;

public class CaptainJIsRSCollect extends SequentialCommandGroup {
    public CaptainJIsRSCollect() {
        addCommands(
                new SetDriveAngle(-90).withTimeout(1.5),
                new ParallelDeadlineGroup(
                        new DrivePathPlannerPath("updatedcollectone", 2, 0.5).andThen(new WaitCommand(0.2)),
                        new CollectBalls(),
                        new SetIntakeState(true)
                ),
                new SetIntakeState(false),
                new SetDriveAngle(-20).withTimeout(0.75),
                new AutoAimShoot(true),
                new SetDriveAngle(149.74).withTimeout(1.5),
                new ParallelDeadlineGroup(
                        new DrivePathPlannerPath("updatedCollectAnother", 2, 0.5).andThen(new WaitCommand(0.2)),
                        new CollectBalls(),
                        new SetIntakeState(true)
                ),
                new SetIntakeState(false),
                new SetDriveAngle(-60).withTimeout(1.5),
                new AutoAimShoot(true)
        );
    }
}
