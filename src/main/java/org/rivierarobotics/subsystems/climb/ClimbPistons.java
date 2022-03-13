package org.rivierarobotics.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbPistons extends SubsystemBase {
    private static ClimbPistons pistons;
    public static ClimbPistons getInstance() {
        if (pistons == null) {
            pistons = new ClimbPistons();
        }
        return pistons;
    }

    private ClimbPistons() {
    }

    public boolean isPistonSet(ClimbPositions climbModule) {
        return climbModule.piston.getState();
    }

    public void setPiston(ClimbPositions climbModule, boolean isEngaged) {
        climbModule.piston.set(isEngaged);
    }

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
}
