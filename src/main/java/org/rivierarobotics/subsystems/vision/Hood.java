package org.rivierarobotics.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    public static Hood hood;

    public static Hood getInstance() {
        if (hood == null) {
            hood = new Hood();
        }
        return hood;
    }

}
