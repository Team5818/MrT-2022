package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class Collect extends CommandBase{
    private final boolean isIntake;
    private final Intake intake;

    public Collect(boolean isIntake) {
        intake = Intake.getInstance();
        this.isIntake = isIntake;
    }

    @Override
    public void initialize() {
        intake.setIntakeState(isIntake);
        intake.setVelocity(123);
    }

    @Override
    public boolean isFinished() {
        intake.setIntakeState(false);
        intake.setVelocity(0);
        return false;
    }


}
