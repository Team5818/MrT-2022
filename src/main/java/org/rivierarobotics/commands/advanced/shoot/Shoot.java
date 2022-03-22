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

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.rivierarobotics.commands.basic.collect.SetBeltAndMiniwheelVoltage;
import org.rivierarobotics.commands.basic.collect.SetBeltVoltage;
import org.rivierarobotics.commands.basic.collect.SetMiniwheelVoltage;
import org.rivierarobotics.commands.basic.shoot.FlywheelTest;
import org.rivierarobotics.commands.basic.shoot.SetFloppaLimelight;
import org.rivierarobotics.commands.basic.shoot.SetFloppaPosition;
import org.rivierarobotics.commands.basic.shoot.SetFlywheelSpeed;
import org.rivierarobotics.subsystems.intake.IntakeBelt;
import org.rivierarobotics.subsystems.shoot.FloppaActuator;
import org.rivierarobotics.subsystems.shoot.FloppaFlywheels;
import org.rivierarobotics.subsystems.shoot.ShooterLocations;

public class Shoot extends SequentialCommandGroup {
    private static final double SHOOT_BELT_VOLTAGE = -7;
    private static final double SHOOT_MINIWHEEL_VOLTAGE = 5;

    public Shoot(){
        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
//                                new WaitUntilCommand(() -> FloppaFlywheels.getInstance().flywheelsWithinTolerance(800)).withTimeout(2),
                                new SetBeltVoltage(SHOOT_BELT_VOLTAGE),
                                new SetMiniwheelVoltage(SHOOT_MINIWHEEL_VOLTAGE),
                                new WaitCommand(1.5)
                        ),
                        new FlywheelTest()
                )
        );
    }

    public Shoot(ShooterLocations locations) {
        this(locations.flyWheelSpeed, locations.floppaAngle);
    }

    public Shoot(double speed, double flywheelAngle) {
        addCommands(
                new ParallelDeadlineGroup(
                        new WaitCommand(1.5),
                        new SetBeltAndMiniwheelVoltage(SHOOT_BELT_VOLTAGE, SHOOT_MINIWHEEL_VOLTAGE),
                        new SetFloppaPosition(flywheelAngle),
                        new SetFlywheelSpeed(speed)
                )
        );
    }

    public Shoot(boolean isAutoaim) {
        addCommands(
                new ParallelDeadlineGroup(
                        new WaitCommand(1.5),
                        new SetBeltAndMiniwheelVoltage(SHOOT_BELT_VOLTAGE, SHOOT_MINIWHEEL_VOLTAGE),
                        new SetFloppaLimelight(true)
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        FloppaFlywheels.getInstance().setVoltage(0);
        IntakeBelt.getInstance().setBeltVoltage(0);
        IntakeBelt.getInstance().setMiniWheelMotorVoltage(0);
    }
}
