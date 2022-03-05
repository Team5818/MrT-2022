package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class Collect extends CommandBase {
    private final Intake intake;

    public Collect() {
        this.intake = Intake.getInstance();
    }

    @Override
    public void execute() {
        intake.setIntakeState(true);
        intake.setIntakeVoltage(5);
        intake.setBeltVoltage(5);
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            intake.setIntakeState(false);
            intake.setIntakeVoltage(0);
            double initialTime = Timer.getFPGATimestamp();
            double waitTime = 2;

            intake.setIntakeVoltage(-1);
            while(Timer.getFPGATimestamp() - initialTime < waitTime) {
                // Do absolutely nothing
            }
            intake.setIntakeVoltage(0);
        }


    }
}
