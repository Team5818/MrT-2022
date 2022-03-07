package org.rivierarobotics.commands.limelight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.vision.Hood;

public class SetFlywheelSpeed extends InstantCommand {
    private final Hood hood;
    private final double speed;

    public SetFlywheelSpeed(double speed) {
        this.hood = Hood.getInstance();
        this.speed = speed;
    }

    @Override
    public void initialize() {
        hood.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        hood.setSpeed(0);
    }
}
