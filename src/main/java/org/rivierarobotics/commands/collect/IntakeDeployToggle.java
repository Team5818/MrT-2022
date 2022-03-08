package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.Intake;

public class IntakeDeployToggle extends CommandBase {
    private final Intake intake;
    private final boolean targetState;

    public IntakeDeployToggle(){
        this.intake = Intake.getInstance();
        this.targetState = intake.getIntakeState();
    }

    @Override
    public void initialize() {
        intake.setIntakeState(true);
    }

//    @Override
//    public boolean isFinished() {
//        return !intake.canCollect();
//    }

//    @Override
//    public void end(boolean interrupted) {
//        intake.setIntakeState(interrupted);
//    }
}
