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

import edu.wpi.first.wpilibj.DigitalInput;
import org.rivierarobotics.subsystems.MotorIDs;

public enum ClimbPositions {
    LOW(
            ClimbConstants.LOW_RADIANS,
            new Piston(MotorIDs.SOLENOID_LOW_OPEN),
            new Piston(MotorIDs.SOLENOID_LOW_CLOSED),
            new DigitalInput(MotorIDs.LOW_SWITCH_A),
            new DigitalInput(MotorIDs.LOW_SWITCH_B)
    ),
    MID(
            ClimbConstants.MID_RADIANS,
            new Piston(MotorIDs.SOLENOID_MID_OPEN),
            new Piston(MotorIDs.SOLENOID_MID_CLOSED),
            new DigitalInput(MotorIDs.MID_SWITCH_A),
            new DigitalInput(MotorIDs.MID_SWITCH_B)
    ),
    HIGH(
            ClimbConstants.HIGH_RADIANS,
            new Piston(MotorIDs.SOLENOID_HIGH_OPEN),
            new Piston(MotorIDs.SOLENOID_HIGH_CLOSED),
            new DigitalInput(MotorIDs.HIGH_SWITCH_A),
            new DigitalInput(MotorIDs.HIGH_SWITCH_B)
    );

    public final double locationRadians;
    public final Piston pistonOpen;
    public final Piston pistonClosed;
    public final DigitalInput input1;
    public final DigitalInput input2;

    ClimbPositions(double rads, Piston pistonOpen, Piston pistonClosed, DigitalInput input1, DigitalInput input2) {
        this.locationRadians = rads;
        this.pistonOpen = pistonOpen;
        this.pistonClosed = pistonClosed;
        this.input1 = input1;
        this.input2 = input2;
    }
}
