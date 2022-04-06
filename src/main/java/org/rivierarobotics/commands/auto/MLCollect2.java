package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.DriveToClosest;
import org.rivierarobotics.commands.advanced.drive.RotateToTargetOffOfPose;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.advanced.shoot.RotateBall;
import org.rivierarobotics.commands.basic.collect.SetIntakeState;

public class MLCollect2 extends SequentialCommandGroup {
    public MLCollect2(boolean isRight) {
        addCommands(
                new SetIntakeState(true),
                new ParallelDeadlineGroup(
                  new SequentialCommandGroup(
                          new FindBall(isRight), new RotateBall().withTimeout(1), new DriveToClosest().withTimeout(1).andThen(new WaitCommand(2)),
                          new FindBall(isRight), new RotateBall().withTimeout(1), new DriveToClosest().withTimeout(1).andThen(new WaitCommand(2))
                  ).withTimeout(20).withInterrupt(() -> Timer.getMatchTime() >= 12.5),
                  new CollectBalls()
                ),new RotateToTargetOffOfPose(),
                new AutoAimShoot(true));
    }
}
