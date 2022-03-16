package org.rivierarobotics.subsystems.shoot;

public class ShooterConstant {
    public static final double MAX_ACTUATOR_ACCELERATION = 0;
    public static final double MAX_ACTUATOR_VELOCITY = 0;
    public static final float MAX_ACTUATOR_LIMIT = 1;
    public static final float MIN_ACTUATOR_LIMIT = 1;
    public static final double ACTUATOR_GEARING = 125;

    public static final double MAX_FLYWHEEL_ACCELERATION = 0;
    public static final double MAX_FLYWHEEL_VELOCITY = 0;
    public static final int TIMEOUTMS = 60;
    public static final double MOTOR_TICKS = 2048;
    public static final double DEGREES_PER_TICK = 1 / MOTOR_TICKS;
    public ShooterConstant(){

    }
}
