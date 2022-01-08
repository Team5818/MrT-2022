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

package org.rivierarobotics.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.commands.drive.SetDriveVoltage;
import org.rivierarobotics.commands.drive.SetWheelbaseAngle;

public class ButtonConfiguration {
    public void initTeleop() {
        new JoystickButton(ControlMap.driverLeft, 1).whenPressed(new SetDriveVoltage(7));
        new JoystickButton(ControlMap.driverLeft, 2).whenPressed(new SetWheelbaseAngle(-90).withTimeout(4));
        new JoystickButton(ControlMap.driverRight, 1).whenPressed(new SetWheelbaseAngle(90).withTimeout(4));
        new JoystickButton(ControlMap.driverRight, 2).whenPressed(new SetWheelbaseAngle(180).withTimeout(4));
    }
}
