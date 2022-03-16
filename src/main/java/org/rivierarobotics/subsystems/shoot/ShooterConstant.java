package org.rivierarobotics.subsystems.shoot;

public class ShooterConstant {
    //FloppaActuator
    public static final double MAX_ACTUATOR_ACCELERATION = 0;
    public static final double MAX_ACTUATOR_VELOCITY = 0;
    public static final double ACTUATOR_ZERO_TICKS = 0.0;
    public static final float MAX_ACTUATOR_TICKS = 10000;
    public static final float MIN_ACTUATOR_TICKS = -10000;
    public static final double ACTUATOR_GEARING = 125;
    //FloppaFlywheels
    public static final double MAX_FLYWHEEL_ACCELERATION = 0;
    public static final double MAX_FLYWHEEL_VELOCITY = 0;
    public static final double MOTOR_TICKS = 2048;
    public static final double DEGREES_PER_TICK = 1 / MOTOR_TICKS;
    //Universal
    public static final int TIMEOUTMS = 60;

    private ShooterConstant() {

    }
}
