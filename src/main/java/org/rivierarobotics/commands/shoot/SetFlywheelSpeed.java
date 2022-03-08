package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.vision.Floppas;

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
    public void end(boolean interrupted) {
        floppas.setSpeed(0);
    }
}
