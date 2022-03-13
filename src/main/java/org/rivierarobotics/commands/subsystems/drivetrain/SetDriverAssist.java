package org.rivierarobotics.commands.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class SetDriverAssist extends InstantCommand {
    private final boolean useDriverAssist;
    public SetDriverAssist(boolean useDriverAssist) {
        this.useDriverAssist = useDriverAssist;
    }

    @Override
    public void initialize() {
        DriveTrain.getInstance().setUseDriverAssist(useDriverAssist);
    }
}
