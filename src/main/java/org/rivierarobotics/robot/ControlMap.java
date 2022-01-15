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

import edu.wpi.first.wpilibj.Joystick;

public class ControlMap {
    private static ControlMap controlMap;
    public static ControlMap getInstance() {
        if(controlMap == null) {
            controlMap = new ControlMap();
        }
        return controlMap;
    }

    public final Joystick driverLeft;
    public final Joystick driverRight;
    public final Joystick coDriverLeft;
    public final Joystick coDriverRight;
    public final Joystick driverButtons;
    public final Joystick coDriverButtons;

    private ControlMap() {
        driverLeft = new Joystick(0);
        driverRight = new Joystick(1);
        coDriverLeft = new Joystick(2);
        coDriverRight = new Joystick(3);
        driverButtons = new Joystick(4);
        coDriverButtons = new Joystick(5);
    }

}
