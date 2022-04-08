package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.DrivePathPlannerPath;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.basic.collect.SetIntakeState;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;

public class Fast3Ball extends SequentialCommandGroup {

    public Fast3Ball(){
        addCommands(
                new SetIntakeState(true),
                new AutoAimShoot(true).withTimeout(1.6),
                new ParallelDeadlineGroup(
                        new DrivePathPlannerPath("fiveBallStart", 9, 5),
                        new CollectBalls()
                ),
                new SetDriveAngle(-47).withTimeout(1.25),
                new AutoAimShoot(true),
                new MLCollect2(true)
        );
    }
}
