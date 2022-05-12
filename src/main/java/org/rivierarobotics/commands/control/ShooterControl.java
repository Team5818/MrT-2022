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

package org.rivierarobotics.commands.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.robot.ControlMap;
import org.rivierarobotics.subsystems.shoot.FloppaActuator;
import org.rivierarobotics.subsystems.shoot.ShooterConstant;

public class ShooterControl extends CommandBase {
    private static final double MAX_VOLTAGE = 10;
    private final FloppaActuator floppasActuator;
    private final Joystick joystick;

    public ShooterControl() {
        this.floppasActuator = FloppaActuator.getInstance();
        this.joystick = ControlMap.CO_DRIVER_RIGHT;
        addRequirements(floppasActuator);
    }

    @Override
    public void execute() {
        var voltage = MathUtil.fitDeadband(joystick.getY()) * MAX_VOLTAGE;
        if (floppasActuator.getAngle() > ShooterConstant.MAX_ACTUATOR_TICKS && voltage > 0) {
            floppasActuator.setVoltage(0);
        } else if (floppasActuator.getAngle() < ShooterConstant.MIN_ACTUATOR_TICKS && voltage < 0) {
            floppasActuator.setVoltage(0);
        } else {
            floppasActuator.setVoltage(voltage);
        }
    }
}
