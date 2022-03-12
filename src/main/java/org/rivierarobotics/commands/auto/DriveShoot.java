package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.drive.SetDriveAngle;
import org.rivierarobotics.commands.shoot.AutoAimShoot;

public class DriveShoot extends SequentialCommandGroup {
    public DriveShoot(boolean isRight) {
        addCommands(
                new DrivePath("back"),
                new SetDriveAngle(isRight ? 50 : -90).withTimeout(3),
                new AutoAimShoot(true)
        );
    }
}
