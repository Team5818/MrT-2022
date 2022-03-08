package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.vision.Floppas;

public class SetShooterVoltage extends CommandBase {
    private final double v;

    public SetShooterVoltage(double voltage) {
        this.v = voltage;
        addRequirements(Floppas.getInstance());
    }

    @Override
    public void execute() {
        Floppas.getInstance().setActuatorVoltage(v);
    }
}
