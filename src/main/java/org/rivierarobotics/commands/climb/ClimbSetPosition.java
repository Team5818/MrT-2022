package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.Climb;

public class ClimbSetPosition extends CommandBase {
    private final Climb climb;
    private final double angle;

    public ClimbSetPosition(double angle) {
        this.climb = Climb.getInstance();
        this.angle = angle;
        addRequirements(this.climb);
    }

    @Override
    public void execute() {
        climb.setPosition(angle);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(climb.getAngle(), angle, 1);
    }
}
