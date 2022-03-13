package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.subsystems.shoot.Floppas;

public class EjectOne extends SequentialCommandGroup {
    public EjectOne() {
        addCommands(
                new InstantCommand(() -> Floppas.getInstance().setSpeed(Floppas.getInstance().getTargetV())).andThen(new WaitCommand(2)),
                new InstantCommand(() -> Intake.getInstance().setVoltages(-11, 0)),
                new WaitCommand(0.15),
                new InstantCommand(() -> Intake.getInstance().setVoltages(0, 0)),
                new InstantCommand(() -> Floppas.getInstance().setSpeed(0))
        );
    }

}
