package org.rivierarobotics.commands.LLCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.vision.*;

public class TrackGoal extends CommandBase {
    private final Hood hood;
    private final Limelight limelight;

    public TrackGoal() {
        this.hood = Hood.getInstance();
        this.limelight = Limelight.getInstance();
        //do not require limelight, it doesn't matter and will make things hard for dt aiming later
        addRequirements(hood);
    }

    @Override
    public void execute() {
        if (limelight.getDetected()) {
            hood.setAngle(limelight.getTy());
        }
    }
}
