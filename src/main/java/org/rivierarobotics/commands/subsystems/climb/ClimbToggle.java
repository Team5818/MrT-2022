package org.rivierarobotics.commands.subsystems.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.climb.ClimbDepreciated;

public class ClimbToggle extends InstantCommand {
    private final ClimbDepreciated climb;

    public ClimbToggle(){
        this.climb = ClimbDepreciated.getInstance();
    }

    @Override
    public void initialize() {
        climb.setPlay(!climb.getPlay());
    }
}
