package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.climb.Climb;

public class ClimbZero extends CommandBase {
    private Climb climb;

    public ClimbZero (){
        climb = Climb.getInstance();
        this.addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setOffset();
    }
}
