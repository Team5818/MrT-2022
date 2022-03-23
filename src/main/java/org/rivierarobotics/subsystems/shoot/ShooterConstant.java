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

package org.rivierarobotics.subsystems.shoot;

public class ShooterConstant {
    //FloppaActuator
    public static final double MAX_ACTUATOR_ACCELERATION = 1800;
    public static final double MAX_ACTUATOR_VELOCITY = 1800;
    public static final float ACTUATOR_ZERO_TICKS = -13.14f;
    public static final float MAX_ACTUATOR_TICKS = 22.3f + ACTUATOR_ZERO_TICKS;
    public static final float MIN_ACTUATOR_TICKS = -8.56f + ACTUATOR_ZERO_TICKS;
    public static final double ACTUATOR_GEARING = 125;
    //FloppaFlywheels
    public static final double MAX_FLYWHEEL_ACCELERATION = 0;
    public static final double MAX_FLYWHEEL_VELOCITY = 0;
    public static final double MOTOR_TICKS = 2048;
    public static final double DEGREES_PER_TICK = 1 / MOTOR_TICKS;
    //Universal
    public static final int TIMEOUTMS = 60;

    private ShooterConstant() {

    }
}
