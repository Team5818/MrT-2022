package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.Intake;

public class CollectToggle extends CommandBase {
    private final Intake intake;
    private final double collectVoltage = 8.0;
    private final boolean targetPositive;
    private final boolean useIntake;
    private final boolean useRollers;

    public CollectToggle(boolean isPositive, boolean useIntake, boolean useRollers){
        this.intake = Intake.getInstance();
        this.targetPositive = isPositive;
        this.useIntake = useIntake;
        this.useRollers = useRollers;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setVoltages(useIntake ? (targetPositive ? collectVoltage : - collectVoltage) : 0, useRollers? (targetPositive ? collectVoltage : - collectVoltage) : 0);
    }

    @Override
    public boolean isFinished() {
        return !intake.canCollect();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            //intake.setVoltages(0,0);
            return;
        }
        intake.setVoltages(0,0);
    }
}
