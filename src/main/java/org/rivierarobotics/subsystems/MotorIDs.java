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

package org.rivierarobotics.subsystems;

public class MotorIDs {
    //Drive Train Motors
    public static final int FRONT_RIGHT_DRIVE = 1;
    public static final int FRONT_RIGHT_STEER = 2;
    public static final int FRONT_LEFT_DRIVE = 3;
    public static final int FRONT_LEFT_STEER = 4;
    public static final int BACK_LEFT_DRIVE = 5;
    public static final int BACK_LEFT_STEER = 6;
    public static final int BACK_RIGHT_DRIVE = 7;
    public static final int BACK_RIGHT_STEER = 8;
    //climb ids
    public static final int SOLENOID_LOW = 0;
    public static final int SOLENOID_MID = 1;
    public static final int SOLENOID_HIGH = 2;
    public static final int CLIMB_ROTATE = 10;


    private MotorIDs() {
    }

}