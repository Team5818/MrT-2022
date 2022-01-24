package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.climb.Climb;

//TODO: Make instant command, remove isFinished
public class OpenAllPistons extends CommandBase {
    private final Climb climb;

    public OpenAllPistons() {
        this.climb = Climb.getInstance();
    }

    @Override
    public void execute() {
        climb.openAllPistons();
    }

    @Override
    public boolean isFinished(){
        return Climb.getClimbPistonsMap().get(Climb.Position.LOW).getState();
    }
}
