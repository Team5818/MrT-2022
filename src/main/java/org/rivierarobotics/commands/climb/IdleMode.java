package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.climb.Climb;

public class IdleMode extends CommandBase {
    private Climb climb;
    private boolean coast;

    public IdleMode(boolean coast) {
        this.climb = Climb.getInstance();
        this.coast = coast;
        this.addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setCoast(coast);
    }
}
