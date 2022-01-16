package org.rivierarobotics.subsystems.climb;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public enum Pistons {

    LOW(0), MID(1), HIGH(2);
    public int id;

    Pistons(int id) {
        this.id = id;
    }

}
