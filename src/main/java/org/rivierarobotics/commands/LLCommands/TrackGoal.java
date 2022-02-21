package org.rivierarobotics.commands.LLCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.vision.Hood;

public class TrackGoal extends CommandBase {
    private final Hood hood;

    public TrackGoal() {
        this.hood = Hood.getInstance();
    }

}
