package org.rivierarobotics.util;

import edu.wpi.first.wpilibj.Timer;

public class RRTimer {
    private double startTime;
    private double absoluteStart;
    private double waitTime;
    public RRTimer(double waitTime) {
        this.startTime = Timer.getFPGATimestamp();
        this.waitTime = waitTime;
        this.absoluteStart = waitTime;
    }

    public boolean finished() {
        return Timer.getFPGATimestamp() - startTime >= this.waitTime;
    }
    public boolean finished(double waitTime) {
        return Timer.getFPGATimestamp() - startTime >= waitTime;
    }

    public void reset() {
        this.startTime = Timer.getFPGATimestamp();
    }

}
