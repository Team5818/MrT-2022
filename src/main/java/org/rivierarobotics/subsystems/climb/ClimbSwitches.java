package org.rivierarobotics.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSwitches extends SubsystemBase {
    private static ClimbSwitches climbSwitches;
    public static ClimbSwitches getInstance() {
        if (climbSwitches == null) {
            climbSwitches = new ClimbSwitches();
        }
        return climbSwitches;
    }

    private ClimbSwitches() {
    }

    public boolean isSwitchSet(ClimbPositions climbModule) {
        return !climbModule.input1.get() || !climbModule.input2.get();
    }


}
