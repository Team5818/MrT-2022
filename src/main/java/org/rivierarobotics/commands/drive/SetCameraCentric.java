package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class SetCameraCentric extends CommandBase {
    private final DriveTrain driveTrain;

    public SetCameraCentric(){
        this.driveTrain = DriveTrain.getInstance();
    }

    @Override
    public void execute() {
        driveTrain.setFieldCentric(false);
    }

//    @Override
//    public boolean isFinished() {
//        driveTrain.setFieldCentric(true);
//        return super.isFinished();
//    }

        @Override
    public void end(boolean interrupted) {
        driveTrain.setFieldCentric(interrupted);
    }
}
