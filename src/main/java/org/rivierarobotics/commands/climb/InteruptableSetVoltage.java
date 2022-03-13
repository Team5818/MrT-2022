package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.climb.ClimbDepreciated;

public class
InteruptableSetVoltage extends CommandBase {

    private final double modifier;
    private final double voltage;
    private ClimbDepreciated climb;

    public InteruptableSetVoltage(boolean reversed, double voltage) {
        this.modifier = reversed ? 1 : -1;
        this.voltage = voltage;
        this.climb = ClimbDepreciated.getInstance();
        addRequirements(climb);
    }
    @Override
    public void execute() {
        if (climb.getPlay()) {
            climb.setVoltage(voltage * modifier);
        } else {
            climb.setVoltage(0);
        }
    }
}
