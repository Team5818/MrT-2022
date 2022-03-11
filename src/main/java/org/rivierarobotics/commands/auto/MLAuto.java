package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.collect.CollectToggle;
import org.rivierarobotics.commands.collect.DriveToClosest;
import org.rivierarobotics.commands.collect.IntakeDeployToggle;
import org.rivierarobotics.commands.collect.SetIntakeState;
import org.rivierarobotics.commands.drive.SetDriveAngle;
import org.rivierarobotics.commands.drive.SetDriveTargetAngle;
import org.rivierarobotics.commands.shoot.AutoAimShoot;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class MLAuto extends SequentialCommandGroup {
    public MLAuto() {
        addCommands(
                new SetDriveTargetAngle(-180),
                new SetIntakeState(true),
                new ParallelDeadlineGroup(
                        new DrivePath("mlauto/mlstart").andThen(new WaitCommand(1)),
                        new CollectToggle(false,true,true)
                ),
                new SetDriveAngle(-70).withTimeout(1.5),
                new AutoAimShoot(),
                new SetDriveTargetAngle(-180),
                new ParallelDeadlineGroup(
                        new DrivePath("mlauto/mlend").andThen(new WaitCommand(1)),
                        new CollectToggle(false,true,true)
                ),
                new SetDriveAngle(-50).withTimeout(1.5),
                new AutoAimShoot(),
                new ParallelDeadlineGroup(
                        new DriveToClosest().alongWith(new WaitCommand(1)),
                        new CollectToggle(false,true,true)
                ),
                new AutoAimShoot()
        );
    }
}
