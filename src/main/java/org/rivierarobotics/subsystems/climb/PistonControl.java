package org.rivierarobotics.subsystems.climb;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PistonControl extends SubsystemBase {

    public static PistonControl getInstance() {
        return pistonControl;
    }

    private static PistonControl pistonControl;

    private static Solenoid low;
    private static Solenoid mid;
    private static Solenoid high;

    public PistonControl() {
        this.low = new Solenoid(PneumaticsModuleType.CTREPCM, Pistons.LOW.id);
        this.mid = new Solenoid(PneumaticsModuleType.CTREPCM, Pistons.MID.id);
        this.high = new Solenoid(PneumaticsModuleType.CTREPCM, Pistons.HIGH.id);
    }

    private Solenoid pistonFor(Pistons piston) {
        switch (piston) {
            case LOW:
                return low;
            case MID:
                return mid;
            case HIGH:
                return high;
            default:
                return null;
        }
    }


    public void set(Pistons piston, boolean isOpen) {
        pistonFor(piston).set(isOpen);
    }

    public boolean get(Pistons piston) {
        return pistonFor(piston).get();
    }

}
