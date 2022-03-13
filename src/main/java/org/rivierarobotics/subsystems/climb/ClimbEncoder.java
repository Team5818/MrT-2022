package org.rivierarobotics.subsystems.climb;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.rivierarobotics.subsystems.MotorIDs.CLIMB_ENCODER;

public class ClimbEncoder extends SubsystemBase {
    private static  ClimbEncoder climbEncoder;
    public static ClimbEncoder getInstance() {
        if (climbEncoder == null) {
            climbEncoder = new ClimbEncoder();
        }
        return climbEncoder;
    }

    private final DutyCycleEncoder encoder;

    private ClimbEncoder() {
        this.encoder = new DutyCycleEncoder(CLIMB_ENCODER);
        this.encoder.setDistancePerRotation(2 * Math.PI);
    }

    public double getAngle() {
        return encoder.getDistance();
    }

    public void setOffset() {
        encoder.reset();
    }
}
