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

package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.robot.ControlMap;
import org.rivierarobotics.subsystems.shoot.Floppas;

public class ShooterControl extends CommandBase {
    private final Floppas floppas;
    private final Joystick joystick;
    private final double MaxVoltage = 10;

    public ShooterControl(){
        this.floppas = Floppas.getInstance();
        this.joystick = ControlMap.CO_DRIVER_RIGHT;
        addRequirements(floppas);
    }

    @Override
    public void execute() {
        var voltage = MathUtil.fitDeadband(joystick.getY()) * MaxVoltage;
        floppas.setActuatorVoltage(voltage);
    }
}
