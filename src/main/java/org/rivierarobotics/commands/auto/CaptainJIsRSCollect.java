package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.basic.collect.SetIntakeState;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;

public class CaptainJIsRSCollect extends SequentialCommandGroup {
    public CaptainJIsRSCollect() {
        addCommands(
                new SetDriveAngle(-90).withTimeout(1.5),
                new ParallelDeadlineGroup(
                        new DrivePathPlannerPath("updatedcollectone", 2, 0.5).andThen(new WaitCommand(0.2)),
                        new CollectBalls(),
                        new SetIntakeState(true)
                ),
                new SetIntakeState(false),
                new SetDriveAngle(-20).withTimeout(0.75),
                new AutoAimShoot(true),
                new SetDriveAngle(149.74).withTimeout(1.5),
                new ParallelDeadlineGroup(
                        new DrivePathPlannerPath("updatedCollectAnother", 2, 0.5).andThen(new WaitCommand(0.2)),
                        new CollectBalls(),
                        new SetIntakeState(true)
                ),
                new SetIntakeState(false),
                new SetDriveAngle(-60).withTimeout(1.5),
                new AutoAimShoot(true)
        );
    }
}
