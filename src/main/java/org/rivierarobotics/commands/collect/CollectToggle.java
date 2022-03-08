package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.subsystems.intake.Intake;

public class CollectToggle extends CommandBase {
    private final Intake intake;
    private final double collectVoltage = 12;
    private final boolean targetPositive;
    private final boolean useIntake;
    private final boolean useRollers;

    public CollectToggle(boolean isPositive, boolean useBelts, boolean useRollers){
        this.intake = Intake.getInstance();
        this.targetPositive = isPositive;
        this.useIntake = useBelts;
        this.useRollers = useRollers;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setVoltages(useIntake ? (targetPositive ? collectVoltage : - collectVoltage) : 0, useRollers? (targetPositive ? collectVoltage : - collectVoltage) : 0);
    }

    @Override
    public boolean isFinished() {
        if (!intake.canCollect()){
            intake.setIsFull(true);
        }
        return !intake.canCollect();
    }

    @Override
    public void end(boolean interrupted) {
        if (intake.getIsFull()) {
            double start = Timer.getFPGATimestamp();
            double timeInterval = 0.25;
            while (Timer.getFPGATimestamp() < start + timeInterval){
                intake.setVoltages(8,0);
            }
            intake.setVoltages(0,0);
            intake.setIsFull(false);
            return;
        } else if (interrupted) {
            intake.setVoltages(0,0);
        }
        //intake.setVoltages(0,0);
    }
}
