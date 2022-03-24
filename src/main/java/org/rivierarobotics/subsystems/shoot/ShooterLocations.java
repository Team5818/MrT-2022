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

package org.rivierarobotics.subsystems.shoot;

public enum ShooterLocations {
    LAUNCHPAD_A(0, 0, 0),
    LAUNCHPAD_B(1,  0, 0),
    LOW_GOAL(60, 0, 0),
    FENDER(110, 0 - 0.15, 0);

    public final double flyWheelSpeed;
    public final double floppaAngle;
    public final double driveAngle;

    ShooterLocations(double flyWheelSpeed, double floppaAngle, double driveAngle) {
        this.flyWheelSpeed = flyWheelSpeed;
        double floppaRaw = FloppaActuator.convertAngleToTicks(floppaAngle);
        //comment out this check if it causes problems while creating locations
        if (!(ShooterConstant.MIN_ACTUATOR_TICKS < floppaRaw && floppaRaw < ShooterConstant.MAX_ACTUATOR_TICKS)) {
            throw new RuntimeException("floppa Angle out of bounds");
        }
        this.floppaAngle = floppaAngle;
        this.driveAngle = driveAngle;
    }
}
