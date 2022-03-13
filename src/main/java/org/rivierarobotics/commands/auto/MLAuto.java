package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.collect.CollectToggle;
import org.rivierarobotics.commands.drive.DrivePath;
import org.rivierarobotics.commands.shoot.AutoAimShoot;
import org.rivierarobotics.commands.subsystems.drivetrain.SetDriveAngle;
import org.rivierarobotics.commands.subsystems.drivetrain.SetDriveTargetAngle;
import org.rivierarobotics.commands.subsystems.intake.SetIntakeState;

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
                new AutoAimShoot()
        );
    }
}
