package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.collect.CollectToggle;
import org.rivierarobotics.commands.collect.IntakeDeployToggle;
import org.rivierarobotics.commands.collect.SetIntakeState;
import org.rivierarobotics.commands.drive.SetDriveAngle;
import org.rivierarobotics.commands.shoot.AutoAimShoot;
import org.rivierarobotics.commands.shoot.Shoot;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class OneBallSimpleAuto extends SequentialCommandGroup {
    public OneBallSimpleAuto(boolean rightSide) {
        if(!rightSide) {
            addCommands(
                    new SetDriveAngle(-135).withTimeout(3),
                    new AutoAimShoot().withTimeout(5),
                    new SetDriveAngle(-255).withTimeout(2),
                    new ParallelDeadlineGroup(
                            new DrivePath("simplediag").andThen(new WaitCommand(2)),
                            new SetIntakeState(true),
                            new CollectToggle(false, true, true)
                    ),
                    new SetDriveAngle(-135).withTimeout(2),
                    new AutoAimShoot().withTimeout(5)
            );
        } else {
            addCommands(
                    new SetDriveAngle(-45).withTimeout(3),
                    new AutoAimShoot().withTimeout(5),
                    new SetDriveAngle(-205).withTimeout(2),
                    new ParallelDeadlineGroup(
                            new DrivePath("simplediag2").andThen(new WaitCommand(2)),
                            new SetIntakeState(true),
                            new CollectToggle(false, true,true)
                    ),
                    new SetDriveAngle(-35).withTimeout(2),
                    new AutoAimShoot().withTimeout(5)
            );
        }
    }
}
