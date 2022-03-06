package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.Intake;

public class collectToggle extends CommandBase {
    private final Intake intake;
    private final double intakeVoltage = 2.0;

    public collectToggle(boolean isPositive){
        this.intake = Intake.getInstance();
    }



}
