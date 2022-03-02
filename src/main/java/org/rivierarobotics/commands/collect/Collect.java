package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class Collect extends CommandBase{
    private final Intake intake;

    public Collect() {
        this.intake = Intake.getInstance();
    }

    @Override
    public void execute() {
        intake.setIntakeState(true);
        intake.setVoltage(4);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeState(!interrupted);
        intake.setVoltage(0);
    }
}
