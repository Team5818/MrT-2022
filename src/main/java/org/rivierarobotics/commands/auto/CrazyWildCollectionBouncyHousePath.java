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
                new SetDriveAngle(-20).withTimeout(0.3),
                new AutoAimShoot().withTimeout(1.4),
                new SetDriveAngle(-90).withTimeout(0.5),
                new ParallelDeadlineGroup(
                  new DrivePathPlannerPath("fiveBallStart", 5, 1.2),
                  new CollectBalls()
                ),
                new SetDriveAngle(-47).withTimeout(1.2),
                new AutoAimShoot(),
                new ParallelDeadlineGroup(
                        new DrivePathPlannerPath("fiveBallEnd", 7, 2),
                        new CollectBalls()
                ),
                new AutoAimShoot()
        );
    }
}
