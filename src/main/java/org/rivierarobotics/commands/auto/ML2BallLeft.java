package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.DrivePathPlannerPath;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.basic.collect.SetIntakeState;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;

public class ML2BallLeft extends SequentialCommandGroup {

    public ML2BallLeft() {
        super(
                new SetIntakeState(true),
                new ParallelDeadlineGroup(
                        new DrivePathPlannerPath("2BallMLBack", 5, 2),
                        new CollectBalls()
                ),
                new SetDriveAngle(-135),
                new SetIntakeState(false),
                new AutoAimShoot(true),
                new MLCollect2(true)
        );
    }
}
