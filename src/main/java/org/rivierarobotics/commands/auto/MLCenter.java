package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.DrivePathPlannerPath;
import org.rivierarobotics.commands.advanced.drive.RotateToTargetOffOfPose;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.basic.collect.SetIntakeState;

public class MLCenter extends SequentialCommandGroup {
    public MLCenter() {
        addCommands(
                new SetIntakeState(true),
                new ParallelCommandGroup(
                        new DrivePathPlannerPath("MLBack", 5, 2),
                        new CollectBalls()
                ),
                new AutoAimShoot(true),
                new MLCollect2(false).andThen(new MLCollect2(false))
        );
    }

}
