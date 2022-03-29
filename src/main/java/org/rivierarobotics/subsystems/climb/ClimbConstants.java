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

package org.rivierarobotics.subsystems.climb;

public class ClimbConstants {
    public static final double LOW_RADIANS = -4.05;
    public static final double MID_RADIANS = 2.64;
    public static final double HIGH_RADIANS = -1.53;
    public static final double MAX_RADS = 4.17;
    public static final double ZERO_TICKS = -184264;
    public static final double MAX_CLIMB_VELOCITY = 30000;
    public static final double MAX_CLIMB_ACCELERATION = 10000;
    public static final int TIMEOUT_MS = 60;
    public static final double GEAR_RATIO = 4000 / 12.0;
    public static final double CLIMB_ENCODER_TICKS = 2048;
    public static final double MOTOR_TICK_TO_ANGLE = (2 * Math.PI) / (CLIMB_ENCODER_TICKS * GEAR_RATIO);
    public static final double MOTOR_ANGLE_TO_TICK = 1 / MOTOR_TICK_TO_ANGLE;

    private ClimbConstants() {
    }
}
