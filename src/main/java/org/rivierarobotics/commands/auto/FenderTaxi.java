package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.advanced.shoot.FenderShot;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;
import org.rivierarobotics.commands.basic.drive.SetDriveVelocity;

public class FenderTaxi extends SequentialCommandGroup {
    public  FenderTaxi() {
        super(
                new FenderShot(),
                new SetDriveVelocity(-1,0,0).withTimeout(2),
                new SetDriveAngle(180)
        );
    }
}
