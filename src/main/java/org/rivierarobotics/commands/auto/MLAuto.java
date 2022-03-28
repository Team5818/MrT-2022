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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.advanced.drive.DrivePathweaverPath;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.basic.collect.SetIntakeState;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;
import org.rivierarobotics.commands.basic.drive.SetDriveTargetAngle;

public class MLAuto extends SequentialCommandGroup {
    public MLAuto() {
        addCommands(
                new SetDriveTargetAngle(-180),
                new SetIntakeState(true),
                new DrivePathweaverPath("mlauto/mlstart"),
                new WaitCommand(1),
                new SetDriveAngle(-70).withTimeout(1.5),
                new AutoAimShoot(),
                new SetDriveTargetAngle(-180),
                new DrivePathweaverPath("mlauto/mlend"),
                new WaitCommand(1),
                new SetDriveAngle(-50).withTimeout(1.5),
                new AutoAimShoot()
        );
    }
}
