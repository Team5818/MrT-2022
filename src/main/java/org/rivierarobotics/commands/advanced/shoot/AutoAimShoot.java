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

package org.rivierarobotics.commands.advanced.shoot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Limelight;

public class AutoAimShoot extends SequentialCommandGroup {
    public AutoAimShoot(boolean isAuto) {
        super(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new Shoot(true)
                        ),
                        new TrackGoal(isAuto)
                ),
                new WaitCommand(0.5),
                () -> Limelight.getInstance().getDetected()
        );
    }

    public AutoAimShoot() {
        this(false);
    }

    @Override
    public void end(boolean interrupted) {
        DriveTrain.getInstance().setUseDriverAssist(false);
    }
}
