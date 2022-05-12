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

package org.rivierarobotics.commands.advanced.shoot;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.basic.collect.SetBeltVoltage;
import org.rivierarobotics.commands.basic.collect.SetMiniwheelVoltage;
import org.rivierarobotics.commands.basic.shoot.SetFloppaPosition;
import org.rivierarobotics.commands.basic.shoot.SetFlywheelSpeed;
import org.rivierarobotics.subsystems.shoot.ShooterLocations;

public class FenderShot extends SequentialCommandGroup {
    public FenderShot() {
        super(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2),
                                        new WaitCommand(0.5)
                                                .andThen(new SetBeltVoltage(ShootAll.SHOOT_BELT_VOLTAGE))
                                                .andThen(new SetMiniwheelVoltage(ShootAll.SHOOT_MINIWHEEL_VOLTAGE)),
                                        new SetFlywheelSpeed(ShooterLocations.FENDER.flywheelSpeed)
                                )
                        ),
                        new SetFloppaPosition(ShooterLocations.FENDER).withTimeout(1)
                ),
                new WaitCommand(1),
                new SetFlywheelSpeed(0),
                new SetBeltVoltage(0),
                new SetMiniwheelVoltage(0)
        );
    }
}
