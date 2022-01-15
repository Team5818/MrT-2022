package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.ClimbLocation;
import org.rivierarobotics.subsystems.climb.Switch;

public class ClimbSetPosition extends CommandBase {
    private final Climb climb;
    private Switch switchObject;
    private final double targetTicks;
    private final ClimbLocation climbLocation;

    public ClimbSetPosition(ClimbLocation climbLocation) {
        this.climb = Climb.getInstance();
        this.climbLocation = climbLocation;
        this.switchObject = climbLocation.switchInstance;
        this.targetTicks = climbLocation.ticks;
        this.addRequirements(this.switchObject);
        this.addRequirements(this.climb);
    }

    @Override
    public void execute() {
        climb.setPosition(targetTicks);
    }

    @Override
    public boolean isFinished() {
        return switchObject.getState();
    }
}
