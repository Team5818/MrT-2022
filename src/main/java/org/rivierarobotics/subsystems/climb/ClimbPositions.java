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

import edu.wpi.first.wpilibj.DigitalInput;

import static org.rivierarobotics.subsystems.MotorIDs.HIGH_SWITCH_A;
import static org.rivierarobotics.subsystems.MotorIDs.HIGH_SWITCH_B;
import static org.rivierarobotics.subsystems.MotorIDs.LOW_SWITCH_A;
import static org.rivierarobotics.subsystems.MotorIDs.LOW_SWITCH_B;
import static org.rivierarobotics.subsystems.MotorIDs.MID_SWITCH_A;
import static org.rivierarobotics.subsystems.MotorIDs.MID_SWITCH_B;
import static org.rivierarobotics.subsystems.MotorIDs.SOLENOID_HIGH;
import static org.rivierarobotics.subsystems.MotorIDs.SOLENOID_LOW;
import static org.rivierarobotics.subsystems.MotorIDs.SOLENOID_MID;
import static org.rivierarobotics.subsystems.climb.ClimbConstants.HIGH_RADIANS;
import static org.rivierarobotics.subsystems.climb.ClimbConstants.LOW_RADIANS;
import static org.rivierarobotics.subsystems.climb.ClimbConstants.MID_RADIANS;

public enum ClimbPositions {

    LOW(LOW_RADIANS, new Piston(SOLENOID_LOW), new DigitalInput(LOW_SWITCH_A), new DigitalInput(LOW_SWITCH_B)),
    MID(MID_RADIANS, new Piston(SOLENOID_MID), new DigitalInput(MID_SWITCH_A), new DigitalInput(MID_SWITCH_B)),
    HIGH(HIGH_RADIANS, new Piston(SOLENOID_HIGH), new DigitalInput(HIGH_SWITCH_A), new DigitalInput(HIGH_SWITCH_B));

    public final double locationRadians;
    public final Piston piston;
    public final DigitalInput input1;
    public final DigitalInput input2;

    ClimbPositions(double rads, Piston piston, DigitalInput input1, DigitalInput input2) {
        this.locationRadians = rads;
        this.piston = piston;
        this.input1 = input1;
        this.input2 = input2;
    }
}
