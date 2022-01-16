package org.rivierarobotics.subsystems.climb;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Switch extends SubsystemBase {

    public static Switch getInstance(Buttons button) {
        switch (button) {
            case LOW:
                if (low == null) {
                    low = new Switch(0);
                }
                return low;
            case MID:
                if (mid == null) {
                    mid = new Switch(1);
                }
                return mid;
            case HIGH:
                if (high == null) {
                    high = new Switch(2);
                }
                return high;
            default:
                return null;
        }
    }

    private static Switch low;
    private static Switch mid;
    private static Switch high;

    public enum Buttons {
        LOW,
        MID,
        HIGH
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
