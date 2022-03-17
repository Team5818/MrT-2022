package org.rivierarobotics.commands.basic.collect;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.intake.IntakeBelt;

public class SetMiniwheelVoltage extends InstantCommand {

    private final IntakeBelt intakeBelt;
    private final double voltage;
    public SetMiniwheelVoltage(double voltage) {
        this.intakeBelt = IntakeBelt.getInstance();
        this.voltage = voltage;
        addRequirements(intakeBelt);
    }

    @Override
    public void initialize() {
        intakeBelt.setMiniWheelMotorVoltage(voltage);
    }
}
