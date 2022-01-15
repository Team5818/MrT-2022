package org.rivierarobotics.subsystems.climb;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Switch extends SubsystemBase {
    private static Switch low;
    private static Switch mid;
    private static Switch high;
    public static Switch getInstanceLow() {
        if (low == null) {
            low = new Switch(0);
        }
        return low;
    }
    public static Switch getInstanceMid() {
        if (mid == null) {
            mid = new Switch(1);
        }
        return mid;
    }
    public static Switch getInstanceHigh() {
        if (high == null) {
            high = new Switch(2);
        }
        return high;
    }
//Verify IDs

    private int id;
    private DigitalInput digitalInput;

    public Switch(int id) {
        this.id = id;
        this.digitalInput = new DigitalInput(id);
    }
    public boolean getState() {
        return !digitalInput.get();
    }
}
