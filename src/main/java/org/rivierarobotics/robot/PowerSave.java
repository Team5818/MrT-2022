package org.rivierarobotics.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.intake.IntakeBelt;
import org.rivierarobotics.subsystems.intake.IntakeRollers;
import org.rivierarobotics.subsystems.shoot.FloppaFlywheels;

public class PowerSave extends InstantCommand {
    private boolean execute;
    private FloppaFlywheels floppas;
    private IntakeBelt belt;

    private IntakeRollers rollers;

    public PowerSave(boolean execute){
        this.execute = execute;
        this.floppas = FloppaFlywheels.getInstance();
        this.belt = IntakeBelt.getInstance();
        this.rollers = IntakeRollers.getInstance();

        addRequirements(floppas);
        addRequirements(belt);
        addRequirements(rollers);
    }

    @Override
    public void initialize(){
        if(execute){
            floppas.leftFlywheel.set(TalonFXControlMode.Disabled, 0);
            floppas.rightFlywheel.set(TalonFXControlMode.Disabled, 0);

            belt.beltMotor.set(TalonSRXControlMode.Disabled, 0);
            belt.miniWheelMotor.set(TalonSRXControlMode.Disabled, 0);

            //rollers.rollerMotor.disable();
        }
    }
}
