package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.collect.CollectToggle;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.subsystems.vision.Floppas;

public class Shoot extends SequentialCommandGroup {

    public Shoot(double speed){
        addCommands(
                new SetFlywheelSpeed(speed),
                new WaitCommand(1),
                new CollectToggle(false, true,false).deadlineWith(new WaitCommand(2)),
                new InstantCommand(() -> Floppas.getInstance().setSpeed(0)),
                new InstantCommand(() -> Intake.getInstance().setVoltages(0,0))

        );
    }

    public Shoot(Floppas.ShooterLocations locations){
        addCommands(
                new SetFlywheelSpeed(locations.flyWheelSpeed),
                new WaitCommand(0.5),
                new CollectToggle(false, true,false),
                new WaitCommand(0.5),
                new SetFlywheelSpeed(0)
        );
    }

}
