package org.rivierarobotics.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbClaws extends SubsystemBase {
    private static ClimbClaws pistons;
    public static ClimbClaws getInstance() {
        if (pistons == null) {
            pistons = new ClimbClaws();
        }
        return pistons;
    }

    private ClimbClaws() {
    }

    public boolean isPistonSet(ClimbPositions climbModule) {
        return climbModule.piston.getState();
    }

    public void setPiston(ClimbPositions climbModule, boolean isEngaged) {
        climbModule.piston.set(isEngaged);
    }

    //TODO: Combine with closeAllPistons and rename to setAllPistons(boolean open)
    public void openAllPistons() {
        ClimbPositions.LOW.piston.set(false);
        ClimbPositions.MID.piston.set(false);
        ClimbPositions.HIGH.piston.set(false);
    }

    public void closeAllPistons() {
        ClimbPositions.LOW.piston.set(true);
        ClimbPositions.MID.piston.set(true);
        ClimbPositions.HIGH.piston.set(true);
    }

    public boolean isSwitchSet(ClimbPositions climbModule) {
        return !climbModule.input1.get() || !climbModule.input2.get();
    }
}
