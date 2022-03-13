package org.rivierarobotics.commands.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class SetDriveTargetAngle extends InstantCommand {
    private final double angle;
    public SetDriveTargetAngle(double angle) {
        this.angle = angle;
    }

    @Override
    public void initialize() {
        DriveTrain.getInstance().setTargetRotationAngle(angle);
    }
}
