package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class DrivePath extends CommandBase {
    private final String path;
    private final DriveTrain driveTrain;

    public DrivePath (String path){
        this.path = path;
        this.driveTrain = DriveTrain.getInstance();
    }

    @Override
    public void initialize() {
        driveTrain.drivePath(path);
    }

    @Override
    public boolean isFinished() {
        return !driveTrain.followHolonomicController();
    }
}

