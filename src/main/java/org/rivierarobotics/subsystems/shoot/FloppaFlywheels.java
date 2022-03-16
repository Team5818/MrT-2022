package org.rivierarobotics.subsystems.shoot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.lib.MotionMagicConfig;
import org.rivierarobotics.lib.MotorUtil;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.subsystems.MotorIDs;

import java.util.ArrayList;

public class FloppaFlywheels extends SubsystemBase {
    private static FloppaFlywheels floppaFlywheels;
    public static FloppaFlywheels getInstance() {
        if(floppaFlywheels == null) {
            floppaFlywheels = new FloppaFlywheels();
        }
        return floppaFlywheels;
    }

    private static final PIDConfig FLYWHEEL_CONFIG = new PIDConfig(0.1,0,0,0);
    private static final MotionMagicConfig MOTION_MAGIC_CONFIG = new MotionMagicConfig(
            new ArrayList<>(),true,
            ShooterConstant.MAX_FLYWHEEL_VELOCITY, ShooterConstant.MAX_FLYWHEEL_ACCELERATION,
            100, 2, ShooterConstant.TIMEOUTMS, 10
    );

    private final WPI_TalonFX leftFlywheel;
    private final WPI_TalonFX rightFlywheel;

    public FloppaFlywheels() {
        this.leftFlywheel = new WPI_TalonFX(MotorIDs.SHOOTER_LEFT);
        this.rightFlywheel = new WPI_TalonFX(MotorIDs.SHOOTER_RIGHT);
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition, FLYWHEEL_CONFIG, MOTION_MAGIC_CONFIG, leftFlywheel, rightFlywheel);
        rightFlywheel.setInverted(true);
    }

    public void setFlywheelSpeed(double speed) {
        leftFlywheel.set(ControlMode.Velocity, speed);
        rightFlywheel.set(ControlMode.Velocity, speed);
    }

    public boolean flywheelsWithinTolerance(double tolerance) {
        boolean left = MathUtil.isWithinTolerance(getLeftFlywheelSpeed(), leftFlywheel.getClosedLoopTarget(), tolerance);
        boolean right = MathUtil.isWithinTolerance(getRightFlywheelSpeed(), rightFlywheel.getClosedLoopTarget(), tolerance);
        return left && right;
    }

    public double getLeftFlywheelSpeed() {
        return leftFlywheel.getSelectedSensorVelocity();
    }

    public double getRightFlywheelSpeed() {
        return rightFlywheel.getSelectedSensorVelocity();
    }

    public void setVoltage(double v) {
        leftFlywheel.setVoltage(v);
        rightFlywheel.setVoltage(v);
    }
}
