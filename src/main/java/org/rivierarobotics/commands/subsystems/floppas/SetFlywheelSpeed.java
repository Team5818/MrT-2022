package org.rivierarobotics.commands.subsystems.floppas;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.shoot.Floppas;

public class SetFlywheelSpeed extends InstantCommand {
    private final Floppas floppas;
    private final double speed;

    public SetFlywheelSpeed(double speed) {
        this.floppas = Floppas.getInstance();
        this.speed = speed;
    }

    @Override
    public void initialize() {
        floppas.setSpeed(speed);
    }

    @Override
    public void execute() {
        floppas.floppaStateSpaceControl();
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            floppas.setSpeed(0);
        }
    }
}
