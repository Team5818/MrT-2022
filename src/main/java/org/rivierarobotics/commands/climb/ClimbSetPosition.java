package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.Switch;

public class ClimbSetPosition extends CommandBase {
    private final Climb climb;
    private Switch switchObject;

    public ClimbSetPosition(Switch switchObject) {
        this.climb = Climb.getInstance();
        this.switchObject = switchObject;
        this.addRequirements(this.switchObject);
        this.addRequirements(this.climb);
    }

    @Override
    public void execute() {
        climb.setPosition(switchObject.getAngle());
    }

    @Override
    public boolean isFinished() {
        return switchObject.getState();
    }
}
