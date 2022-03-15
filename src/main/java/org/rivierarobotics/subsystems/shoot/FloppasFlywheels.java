package org.rivierarobotics.subsystems.shoot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MotionMagicConfig;
import org.rivierarobotics.lib.MotorUtil;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.subsystems.MotorIDs;

import java.util.ArrayList;

public class FloppasFlywheels extends CommandBase {

    private final PIDConfig flywheelConfig;
    private final MotionMagicConfig motionMagicConfig;

    private final WPI_TalonFX leftFlywheel;
    private final WPI_TalonFX rightFLywheel;

    public FloppasFlywheels(){
        this.leftFlywheel = new WPI_TalonFX(MotorIDs.SHOOTER_LEFT);
        this.rightFLywheel = new WPI_TalonFX(MotorIDs.SOLENOID_HIGH);
        rightFLywheel.follow(leftFlywheel);
        rightFLywheel.setInverted(true);

        this.flywheelConfig = new PIDConfig(0.1,0,0,0);
        this.motionMagicConfig = new MotionMagicConfig(
                new ArrayList<>(),true,
                ShooterConstant.MAX_FLYWHEEL_VELOCITY, ShooterConstant.MAX_FLYWHEEL_ACCELERATION,
                100, 2, ShooterConstant.TIMEOUTMS, 10
        );

        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition, flywheelConfig, motionMagicConfig, leftFlywheel);
    }

    public void setFlywheelSpeed(double speed){
        leftFlywheel.set(ControlMode.MotionMagic, speed);
    }

    public void getFlywheelSpeed() {
        leftFlywheel.getSensorCollection().getIntegratedSensorVelocity();
    }
}
