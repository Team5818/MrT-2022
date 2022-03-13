package org.rivierarobotics.subsystems.shoot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooters extends SubsystemBase {
    private static Shooters shooters;
    public static Shooters getInstance() {
        if(shooters == null) {
            shooters = new Shooters();
        }
        return shooters;
    }


    public Shooters() {

    }
}
