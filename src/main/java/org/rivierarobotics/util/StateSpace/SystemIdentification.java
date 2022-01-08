package org.rivierarobotics.util.StateSpace;

public class SystemIdentification {
    public double kS;
    public double kV;
    public double kA;
    public SystemIdentification(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }
}