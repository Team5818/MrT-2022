package org.rivierarobotics.commands.basic.shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.shoot.FloppaActuator;
import org.rivierarobotics.subsystems.shoot.ShooterConstant;

public class SetFloppaZero extends InstantCommand {
    private final FloppaActuator floppaActuator;

    public SetFloppaZero() {
        this.floppaActuator = FloppaActuator.getInstance();
        addRequirements(floppaActuator);
    }

    @Override
    public void initialize() {
        ShooterConstant.ACTUATOR_ZERO_TICKS = (float) floppaActuator.getTicks();
    }
}
