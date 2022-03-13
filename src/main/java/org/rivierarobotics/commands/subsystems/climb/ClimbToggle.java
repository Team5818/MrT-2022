package org.rivierarobotics.commands.subsystems.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.climb.Climb;

public class ClimbToggle extends InstantCommand {
    private final Climb climb;

    public ClimbToggle(){
        this.climb = Climb.getInstance();
    }

    @Override
    public void initialize() {
        climb.setPlay(!climb.getPlay());
    }
}
