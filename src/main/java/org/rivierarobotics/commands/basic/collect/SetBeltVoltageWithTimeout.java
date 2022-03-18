package org.rivierarobotics.commands.basic.collect;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.IntakeBelt;

public class SetBeltVoltageWithTimeout extends CommandBase {

    private final IntakeBelt intakeBelt;
    private final double voltage;
    private final double timeout;
    private double startTime = 0.0;
    public SetBeltVoltageWithTimeout(double voltage, double timeout) {
        this.intakeBelt = IntakeBelt.getInstance();
        this.voltage = voltage;
        this.timeout = timeout;
        addRequirements(intakeBelt);
    }

    @Override
    public void initialize() {
        intakeBelt.setBeltVoltage(voltage);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= timeout;
    }

    @Override
    public void end(boolean interrupted) {
        intakeBelt.setBeltVoltage(0);
    }
}
