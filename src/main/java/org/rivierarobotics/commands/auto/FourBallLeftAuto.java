package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.DrivePathPlannerPath;
import org.rivierarobotics.commands.basic.collect.SetIntakeState;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;
import org.rivierarobotics.commands.basic.shoot.SetFloppaLimelight;

public class FourBallLeftAuto extends SequentialCommandGroup {

    public FourBallLeftAuto() {
        super(
                new TwoBallLeftAuto(),
                new SetIntakeState(true),
                new ParallelDeadlineGroup(
                        new DrivePathPlannerPath("2MoreLeft", 1, 0.5),
                        new CollectBalls()
                ),
                new SetDriveAngle(-90),
                new SetIntakeState(false),
                new SetFloppaLimelight(true),
                new MLCollect2(false)
        );
    }
}
