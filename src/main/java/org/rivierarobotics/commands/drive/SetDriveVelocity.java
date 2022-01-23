package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class SetDriveVelocity extends CommandBase {
    private final DriveTrain driveTrain;
    private final double velocity;

    public SetDriveVelocity(double vel){
        this.driveTrain = DriveTrain.getInstance();
        this.velocity = vel;
    }

    @Override
    public void execute() {
        driveTrain.setSwerveModuleVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted){
        driveTrain.setSwerveModuleVelocity(0);
    }
}
