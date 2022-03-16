package org.rivierarobotics.subsystems.shoot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.lib.shuffleboard.RSTab;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.smartmotion.SparkMotionConfig;
import org.rivierarobotics.util.smartmotion.SparkSmartMotion;


public class FloppasActuator extends SubsystemBase {
    public static FloppasActuator floppasActuator;
    public static FloppasActuator getInstance(){
        if (floppasActuator == null) {
            floppasActuator = new FloppasActuator();
        }
        return floppasActuator;
    }

    private final PIDConfig actuatorConfig;
    private final SparkMotionConfig sparkSmartMotionConfig;
    private final SparkSmartMotion actuatorController;
    private final RSTab tunningTab;

    private final CANSparkMax actuatorMotor;

    public FloppasActuator() {
        this.actuatorMotor = new CANSparkMax(MotorIDs.SHOOTER_ANGLE, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.actuatorConfig = new PIDConfig(0.1, 0, 0,0);
        this.sparkSmartMotionConfig = new SparkMotionConfig(
                true,
                ShooterConstant.MAX_ACTUATOR_VELOCITY, ShooterConstant.MAX_ACTUATOR_ACCELERATION,
                100, 2,
                ShooterConstant.TIMEOUTMS, 10
        );
        this.tunningTab = Logging.robotShuffleboard.getTab("actuator tuning");

        this.actuatorController = new SparkSmartMotion(actuatorMotor, actuatorConfig, sparkSmartMotionConfig, tunningTab);

//        this.actuatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ShooterConstant.MAX_ACTUATOR_LIMIT * );
//        this.actuatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ShooterConstant.MIN_ACTUATOR_LIMIT);

    }

    //takes angle in radians setpoint should be in rotations and adjusted for gearing

    public void setFloppasAngle(double angle){
        var setpoint = angle / (2 * Math.PI) * ShooterConstant.ACTUATOR_GEARING;
        actuatorController.getPidController().setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    //returns angle without gearing values possibly in rotations

    public double getFloppasAngle(){
        return actuatorMotor.getEncoder().getPosition() / ShooterConstant.ACTUATOR_GEARING;
    }

    public void setVoltage(double voltage){
        this.actuatorMotor.setVoltage(voltage);
    }
}
