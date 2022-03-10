package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.vision.Floppas;

public class SetFloppaPosition extends CommandBase {
    private final double flywheelRads;
    private final Floppas floppas;
    public SetFloppaPosition(double flywheelRads) {
        this.flywheelRads = flywheelRads;
        this.floppas = Floppas.getInstance();
    }

    public SetFloppaPosition(Floppas.ShooterLocations preset) {
        this(preset.floppaAngle);
    }

    @Override
    public void initialize() {
        this.floppas.setAngle(flywheelRads);
    }

    @Override
    public void execute() {
        this.floppas.floppaStateSpaceControl();
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(floppas.getAngle(), flywheelRads, 0.1);
    }
}
