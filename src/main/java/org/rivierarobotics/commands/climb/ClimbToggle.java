package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.climb.Climb;

public class ClimbToggle extends CommandBase {
    private final Climb climb;

    public ClimbToggle(){
        this.climb = Climb.getInstance();
    }

    @Override
    public void initialize() {
        climb.setPlay(true);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            climb.setPlay(false);
            return;
        }
    }
}
