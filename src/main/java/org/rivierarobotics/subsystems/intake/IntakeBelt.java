package org.rivierarobotics.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeBelt extends SubsystemBase {
    private static IntakeBelt intakeBelt;
    public static IntakeBelt getInstance() {
        if(intakeBelt == null) {
            intakeBelt = new IntakeBelt();
        }
        return intakeBelt;
    }
}
