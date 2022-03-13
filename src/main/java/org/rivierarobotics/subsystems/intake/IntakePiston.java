package org.rivierarobotics.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePiston extends SubsystemBase {
    private static IntakePiston intakePiston;
    public static IntakePiston getInstance() {
        if(intakePiston == null) {
            intakePiston = new IntakePiston();
        }
        return intakePiston;
    }
}
