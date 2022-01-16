package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.climb.PistonControl;
import org.rivierarobotics.subsystems.climb.Pistons;

public class SetPistonState extends CommandBase {

    private final Pistons piston;
    private final boolean isOpen;
    private final PistonControl pistonControl;

    public SetPistonState(Pistons piston, boolean isOpen) {
        this.piston = piston;
        this.isOpen = isOpen;
        this.pistonControl = PistonControl.getInstance();
    }

    @Override
    public void execute() {
        pistonControl.set(piston, isOpen);
    }

    @Override
    public boolean isFinished() {
        return pistonControl.get(piston) == isOpen;
    }
}
