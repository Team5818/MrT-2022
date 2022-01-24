package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class SetDriveVelocity extends CommandBase {
    private final DriveTrain driveTrain;
    private final double velocityX;
    private final double velocityY;
    private final double rotationVel;

    public SetDriveVelocity(double velocityX, double velyocityY, double rotationVel){
        this.driveTrain = DriveTrain.getInstance();
        this.velocityX = velocityX;
        this.velocityY = velyocityY;
        this.rotationVel = rotationVel;
    }

    @Override
    public void execute() {
        driveTrain.drive(velocityX, velocityY, rotationVel, true);
    }

    @Override
    public void end(boolean interrupted){
        driveTrain.setSwerveModuleVelocity(0);
    }
}
