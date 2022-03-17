package org.rivierarobotics.commands.basic.collect;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.intake.IntakePiston;

public class ToggleIntakeState extends InstantCommand {

    private final IntakePiston piston;
    public ToggleIntakeState() {
        piston = IntakePiston.getInstance();
        addRequirements(piston);
    }

    @Override
    public void initialize() {
        piston.setIntakeState(!piston.getIntakeState());
    }
}
