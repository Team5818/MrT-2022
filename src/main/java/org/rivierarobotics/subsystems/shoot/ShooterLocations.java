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
    FENDER(5250, -5, 0);

    public final double flywheelSpeed;
    public final double floppaAngle;
    public final double driveAngle;

    ShooterLocations(double flywheelSpeed, double floppaAngle, double driveAngle) {
        this.flywheelSpeed = flywheelSpeed;
        this.floppaAngle = floppaAngle;
        this.driveAngle = driveAngle;
    }
}
