package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.collect.CollectToggle;
import org.rivierarobotics.commands.collect.DriveToClosest;
import org.rivierarobotics.commands.collect.IntakeDeployToggle;
import org.rivierarobotics.commands.shoot.AutoAimShoot;
import org.rivierarobotics.subsystems.intake.Intake;

public class MLAuto extends SequentialCommandGroup {
    public MLAuto() {
        addCommands(
                new InstantCommand(() -> {
                    Intake.getInstance().setIntakeState(true);
                }),
                new ParallelDeadlineGroup(
                        new DriveToClosest().alongWith(new WaitCommand(4)),
                        new CollectToggle(true, true, true)
                ),
                new DriveToPoint(10, 10, true, 0),
                new AutoAimShoot()
        );
    }
}
