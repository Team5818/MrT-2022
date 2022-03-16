package org.rivierarobotics.subsystems.shoot;

public enum ShooterLocations {
    LAUNCHPAD_A(0, 0, 0),
    LAUNCHPAD_B(1,  0, 0),
    LOW_GOAL(60, 0, 0),
    FENDER(110, 0 - 0.15, 0);

    public final double flyWheelSpeed;
    public final double floppaAngle;
    public final double driveAngle;

    ShooterLocations(double flyWheelSpeed, double floppaAngle, double driveAngle) {
        this.flyWheelSpeed = flyWheelSpeed;
        double floppaRaw = FloppaActuator.convertAngleToTicks(floppaAngle);
        if (!(ShooterConstant.MIN_ACTUATOR_TICKS <= floppaRaw && floppaRaw <= ShooterConstant.MAX_ACTUATOR_TICKS)) {
            throw new RuntimeException("floppa Angle out of bounds");
        }
        this.floppaAngle = floppaAngle;
        this.driveAngle = driveAngle;
    }
}
