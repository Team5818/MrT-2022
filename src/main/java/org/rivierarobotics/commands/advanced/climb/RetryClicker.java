package org.rivierarobotics.commands.advanced.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.shuffleboard.RobotShuffleboard;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.ClimbClaws;
import org.rivierarobotics.subsystems.climb.ClimbPositions;

public class RetryClicker extends CommandBase {
    private final ClimbClaws climbClaws;
    private final int counterMax;
    private int counter = 0;
    private ClimbPositions positions;

    public RetryClicker( int counterMax, ClimbPositions positions) {
        this.climbClaws = ClimbClaws.getInstance();
        addRequirements(climbClaws);
        this.counterMax = counterMax;
        this.positions = positions;
    }

    @Override
    public void execute() {
        if (climbClaws.isSwitchSet(positions)) {
            counter++;
        } else {
            counter = 0;
        }
        Logging.robotShuffleboard.getTab("Climb").setEntry("counter " + positions.toString(), counter);
    }

    @Override
    public boolean isFinished() {
        return counter >= counterMax;
    }
}
