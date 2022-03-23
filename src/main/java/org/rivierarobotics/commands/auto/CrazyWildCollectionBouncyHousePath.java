package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.advanced.shoot.Shoot;
import org.rivierarobotics.commands.basic.collect.SetIntakeState;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class CrazyWildCollectionBouncyHousePath extends SequentialCommandGroup {
    private static final double initialShootSpeed = 5700;
    private static final double initialShootAngle = 9.36;

    public CrazyWildCollectionBouncyHousePath() {
        addCommands(
                new SetIntakeState(true),
                new Shoot(initialShootSpeed, initialShootAngle),
                new SetDriveAngle(-90).withTimeout(1.5),
                new ParallelDeadlineGroup(
                  new DrivePathPlannerPath("/fiveBall/fiveBallStart", 2.5, 0.7),
                  new CollectBalls()
                ),
                new AutoAimShoot(),
                new ParallelDeadlineGroup(
                        new DrivePathPlannerPath("/fiveBall/fiveBallEnd", 2.5, 0.7),
                        new CollectBalls()
                ),
                new AutoAimShoot()
        );
    }
}
