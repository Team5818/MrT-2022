package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.DrivePathPlannerPath;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.basic.collect.SetIntakeState;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;

public class TwoBallLeftAuto extends SequentialCommandGroup {

    public TwoBallLeftAuto() {
        super(
                new SetIntakeState(true),
                new ParallelDeadlineGroup(
                        new DrivePathPlannerPath("2BallLeft", 1, 0.5),
                        new CollectBalls()
                ),
                new SetDriveAngle(-135).withTimeout(1.5),
                new SetIntakeState(false),
                new AutoAimShoot(true).withTimeout(2)
//                new MLCollect2(true)
        );
    }
}
