package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.DriveToClosest;
import org.rivierarobotics.commands.advanced.drive.RotateToTargetOffOfPose;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.basic.collect.SetIntakeState;

public class MLCollect2 extends SequentialCommandGroup {
    public MLCollect2(boolean isRight) {
        addCommands(
                new SetIntakeState(true),
                new ParallelDeadlineGroup(
                  new SequentialCommandGroup(
                          new FindBall(isRight), new DriveToClosest(),new FindBall(isRight),new DriveToClosest()
                  ).withTimeout(9),
                  new CollectBalls()
                ),
                new RotateToTargetOffOfPose(),
                new AutoAimShoot(true)
                );
    }
}
