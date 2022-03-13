package org.rivierarobotics.commands.subsystems.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.intake.Intake;

public class SetIntakeState extends InstantCommand {
    private final boolean isOpen;
    public SetIntakeState(boolean isOpen) {
        this.isOpen = isOpen;
    }

    @Override
    public void initialize() {
        Intake.getInstance().setIntakeState(isOpen);
    }
}
