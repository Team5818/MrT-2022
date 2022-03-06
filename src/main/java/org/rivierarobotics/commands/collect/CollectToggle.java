package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.Intake;

public class CollectToggle extends CommandBase {
    private final Intake intake;
    private final double collectVoltage = 5.0;
    private final boolean targetPositive;

    public CollectToggle(boolean isPositive){
        this.intake = Intake.getInstance();
        this.targetPositive = isPositive;
    }

    @Override
    public void initialize() {
        if (targetPositive == (Math.signum(intake.getIsPositive()) > 0)) {
            intake.setIsPositive(0);
            end(true);
        } else {
            intake.setIsPositive(targetPositive ? 1. : -1.);
        }
//        } else {
//            intake.setIntakeVoltage( targetPositive ? collectVoltage : -collectVoltage);
//            intake.setBeltVoltage( targetPositive ? collectVoltage : -collectVoltage);
//        }
    }

    @Override
    public void execute() {
        intake.setIntakeVoltage( targetPositive ? collectVoltage : -collectVoltage);
        intake.setBeltVoltage( targetPositive ? collectVoltage : -collectVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeVoltage(0);
        intake.setBeltVoltage(0);
    }
}
