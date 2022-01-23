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

package org.rivierarobotics.subsystems.climb;

//TODO: Move into Climb and delete this file
public enum ClimbLocation {

    LOW(Switch.getInstance(Switch.Buttons.LOW), 0),
    MID(Switch.getInstance(Switch.Buttons.MID), 0),
    HIGH(Switch.getInstance(Switch.Buttons.HIGH), 0);

    //figure out tick values later

    public final Switch switchInstance;
    public final double ticks;

    ClimbLocation(Switch switchInstance, double ticks) {
        this.switchInstance = switchInstance;
        this.ticks = ticks;
    }
}
