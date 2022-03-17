package org.rivierarobotics.commands.basic.collect;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.intake.IntakeBelt;

public class SetBeltAndMiniwheelVoltage extends InstantCommand {

    private final IntakeBelt intakeBelt;
    private final double beltVoltage;
    private final double miniwheelVoltage;
    public SetBeltAndMiniwheelVoltage(double beltVoltage, double miniwheelVoltage) {
        this.intakeBelt = IntakeBelt.getInstance();
        this.beltVoltage = beltVoltage;
        this.miniwheelVoltage = miniwheelVoltage;
        addRequirements(intakeBelt);
    }

    @Override
    public void initialize() {
        intakeBelt.setBeltVoltage(beltVoltage);
        intakeBelt.setMiniWheelMotorVoltage(miniwheelVoltage);
    }
}
