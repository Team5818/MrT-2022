package org.rivierarobotics.subsystems.climb;

public enum ClimbLocation {

    LOW(Switch.getInstance(Switch.Buttons.LOW), 0),
    MID(Switch.getInstance(Switch.Buttons.MID), 0),
    HIGH(Switch.getInstance(Switch.Buttons.HIGH), 0);

    //figure out tick values later

    public final Switch switchInstance;
    public final double ticks;

    ClimbLocation(Switch switchInstance, double ticks) {
        this.switchInstance = switchInstance;
        this.ticks = ticks;
    }
}
