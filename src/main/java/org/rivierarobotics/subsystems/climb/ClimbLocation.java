package org.rivierarobotics.subsystems.climb;

public enum ClimbLocation {

    LOW(Switch.getInstanceLow(), 0),
    MID(Switch.getInstanceMid(), 0),
    HIGH(Switch.getInstanceHigh(), 0);

    //figure out tick values later

    public final Switch switchInstance;
    public final double ticks;

    ClimbLocation(Switch switchInstance, double ticks){
        this.switchInstance = switchInstance;
        this.ticks = ticks;
    }
}
