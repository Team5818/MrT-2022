package org.rivierarobotics.subsystems.intake;

// do not make this a subsystem please thank you
public class IntakeSensors {
    private static IntakeSensors intakeSensors;
    public static IntakeSensors getInstance() {
        if(intakeSensors == null) {
            intakeSensors = new IntakeSensors();
        }
        return intakeSensors;
    }
}
