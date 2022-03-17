package org.rivierarobotics.commands.basic.collect;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.intake.IntakeRollers;

public class SetIntakeVoltage extends InstantCommand {

    private final IntakeRollers intakeRollers;
    private final double voltage;
    public SetIntakeVoltage(double voltage) {
        this.intakeRollers = IntakeRollers.getInstance();
        this.voltage = voltage;
        addRequirements(intakeRollers);
    }

    @Override
    public void initialize() {
        intakeRollers.setRollerVoltage(voltage);
    }
}
