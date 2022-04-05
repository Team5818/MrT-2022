package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.DrivePathPlannerPath;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;

public class TwoBallLeftAuto extends SequentialCommandGroup {

    public TwoBallLeftAuto(){
        super(
            new ParallelDeadlineGroup(
                    new DrivePathPlannerPath("2BallLeft", 7,4 ),
                    new CollectBalls()
            ),
            new SetDriveAngle(45),
            new AutoAimShoot(true)
        );
    }
}
