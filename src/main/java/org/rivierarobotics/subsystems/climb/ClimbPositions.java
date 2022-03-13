package org.rivierarobotics.subsystems.climb;

import edu.wpi.first.wpilibj.DigitalInput;

import static org.rivierarobotics.subsystems.MotorIDs.*;
import static org.rivierarobotics.subsystems.climb.ClimbValues.*;

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
