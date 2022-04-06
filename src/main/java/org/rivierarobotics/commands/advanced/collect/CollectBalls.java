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

package org.rivierarobotics.commands.advanced.collect;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.rivierarobotics.commands.advanced.shoot.EjectCollect;
import org.rivierarobotics.commands.basic.collect.SetIntakeVoltage;
import org.rivierarobotics.subsystems.intake.IntakeBelt;
import org.rivierarobotics.subsystems.intake.IntakeRollers;
import org.rivierarobotics.subsystems.intake.IntakeSensors;

public class CollectBalls extends ConditionalCommand {
    public static final double COLLECT_VOLTAGE = -10;
    private static final double INTAKE_VOLTAGE = -12;
    private static final double MINIWHEEL_VOLTAGE = -7;

    public CollectBalls() {
        super(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> !IntakeSensors.getInstance().canCollect()),
                        new SetIntakeVoltage(INTAKE_VOLTAGE),
                        new EjectCollect(COLLECT_VOLTAGE, MINIWHEEL_VOLTAGE)
                ),
                new WaitCommand(0.1),
            () -> IntakeSensors.getInstance().canCollect()
        );
    }

    @Override
    public void end(boolean interrupted) {
        IntakeBelt.getInstance().setBeltVoltage(0);
        IntakeBelt.getInstance().setMiniWheelMotorVoltage(0);
        IntakeRollers.getInstance().setRollerVoltage(0);
    }
}
