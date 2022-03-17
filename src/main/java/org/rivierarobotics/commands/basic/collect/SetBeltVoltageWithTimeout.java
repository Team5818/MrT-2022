package org.rivierarobotics.commands.basic.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.IntakeBelt;

public class SetBeltVoltageWithTimeout extends CommandBase {

    private final IntakeBelt intakeBelt;
    private final double voltage;
    public SetBeltVoltageWithTimeout(double voltage, double timeout) {
        this.intakeBelt = IntakeBelt.getInstance();
        this.voltage = voltage;
        withTimeout(timeout);
        addRequirements(intakeBelt);
    }

    @Override
    public void initialize() {
        intakeBelt.setBeltVoltage(voltage);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeBelt.setBeltVoltage(0);
    }
}
