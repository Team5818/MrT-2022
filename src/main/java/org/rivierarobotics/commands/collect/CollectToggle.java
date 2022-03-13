package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.subsystems.shoot.Floppas;

public class CollectToggle extends CommandBase {
    private final Intake intake;
    private final Floppas floppas;
    public static final double BELT_VOLTAGE = 8;
    public static final double INTAKE_VOLTAGE = 12;
    private final boolean targetPositive;
    private final boolean useIntake;
    private final boolean useRollers;

    public CollectToggle(boolean isPositive, boolean useBelts, boolean useRollers) {
        this.intake = Intake.getInstance();
        this.targetPositive = isPositive;
        this.useIntake = useBelts;
        this.useRollers = useRollers;
        this.floppas = Floppas.getInstance();
        addRequirements(intake, floppas);
    }

    @Override
    public void initialize() {
        floppas.setBlockSS(true);
        floppas.setAngle(Floppas.ZERO_ANGLE);
    }

    @Override
    public void execute() {
        floppas.setShooterVoltage(-1);
        floppas.floppaStateSpaceControl();
        intake.setVoltages(useIntake ? (targetPositive ? INTAKE_VOLTAGE : -INTAKE_VOLTAGE)
                : 0, useRollers ? (targetPositive ? BELT_VOLTAGE : -BELT_VOLTAGE) : 0);
    }

    @Override
    public boolean isFinished() {
        if (!intake.canCollect()) {
            intake.setIsFull(true);
        }
        return !intake.canCollect();
    }

    @Override
    public void end(boolean interrupted) {
        floppas.setBlockSS(false);
        floppas.setActuatorVoltage(0);
        double start = Timer.getFPGATimestamp();
        double timeInterval = 0.25;
        while (Timer.getFPGATimestamp() < start + timeInterval) {
            intake.setVoltages(8, 0);
        }
        intake.setVoltages(0, 0);
        intake.setIsFull(false);
    }
}
