package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.climb.Piston;

public class SetPistonState extends CommandBase {

    private Piston piston;
    private boolean isOpen;

    public SetPistonState(Piston piston, boolean isOpen) {
        this.piston = piston;
        this.isOpen = isOpen;
    }
    @Override
    public void execute() {
        piston.set(isOpen);
    }
    @Override
    public boolean isFinished() {
        return piston.get() == isOpen;
    }
}
