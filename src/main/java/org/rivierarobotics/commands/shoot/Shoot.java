package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.collect.CollectToggle;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.subsystems.vision.Floppas;

public class Shoot extends SequentialCommandGroup {

    public Shoot(){
        addCommands(
                new InstantCommand(() -> Floppas.getInstance().setSpeed(Floppas.getInstance().getTargetV())),
                new WaitCommand(0.35),
                new InstantCommand(() -> Intake.getInstance().setVoltages(-11, 0)),
                new WaitCommand(2),
                new InstantCommand(() -> Floppas.getInstance().setSpeed(0)),
                new InstantCommand(() -> Intake.getInstance().setVoltages(0,0))
        );
    }

    public Shoot(boolean useFloppas) {
        addCommands(
                new InstantCommand(() -> Floppas.getInstance().setSpeed(Floppas.getInstance().getTargetV())).andThen(new WaitCommand(2)),
                new InstantCommand(() -> Intake.getInstance().setVoltages(-11, 0)),
                new WaitCommand(0.2),
                new InstantCommand( ()-> Intake.getInstance().setVoltages(0,0)),
                new WaitCommand(0.2),
                new InstantCommand(()-> Intake.getInstance().setVoltages(-11,0)),
                new WaitCommand(0.5),
                new InstantCommand(() -> Floppas.getInstance().setSpeed(0)),
                new InstantCommand(() -> Intake.getInstance().setVoltages(0,0))
        );
    }

    public Shoot(Floppas.ShooterLocations locations){
        this(locations.flyWheelSpeed, locations.floppaAngle);
    }

    public Shoot(double speed, double flywheelAngle) {
        addCommands(
                new SetFloppaPosition(flywheelAngle).withTimeout(2),
                new InstantCommand(() -> Floppas.getInstance().setSpeed(speed))
                        .until(() -> Floppas.getInstance().getLeftSpeed() >= speed)
                        .withTimeout(1),
                new InstantCommand(() -> Intake.getInstance().setVoltages(-11, 0)),
                new WaitCommand(0.2),
                new InstantCommand( ()-> Intake.getInstance().setVoltages(0,0)),
                new WaitCommand(0.2),
                new InstantCommand(()-> Intake.getInstance().setVoltages(-11,0)),
                new WaitCommand(0.5),
                new InstantCommand(() -> Floppas.getInstance().setSpeed(0)),
                new InstantCommand(() -> Intake.getInstance().setVoltages(0,0))
        );
    }

}
