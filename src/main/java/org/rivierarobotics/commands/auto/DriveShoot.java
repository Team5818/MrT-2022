package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.drive.DrivePath;
import org.rivierarobotics.commands.shoot.AutoAimShoot;
import org.rivierarobotics.commands.subsystems.drivetrain.SetDriveAngle;

public class DriveShoot extends SequentialCommandGroup {
    public DriveShoot(boolean isRight) {
        addCommands(
                new DrivePath("back"),
                new SetDriveAngle(isRight ? -70 : -135).withTimeout(2),
                new AutoAimShoot(true)
        );
    }
}
