package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.ClimbLocation;
import org.rivierarobotics.subsystems.climb.Switch;

public class ClimbSetPosition extends CommandBase {
    private final Climb climb;
    private final Switch switchObject;
    private final ClimbLocation climbLocation;

    public ClimbSetPosition(ClimbLocation climbLocation) {
        this.climb = Climb.getInstance();
        this.climbLocation = climbLocation;
        this.switchObject = climbLocation.switchInstance;
        this.addRequirements(this.switchObject, this.climb);
    }

    @Override
    public void execute() {
        climb.setPosition(climbLocation.ticks);
    }

    @Override
    public boolean isFinished() {
        return switchObject.getState();
    }
}
