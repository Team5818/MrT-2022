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
}
