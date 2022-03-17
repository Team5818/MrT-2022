package org.rivierarobotics.commands.advanced.collect;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.rivierarobotics.commands.basic.collect.SetBeltAndMiniwheelVoltage;
import org.rivierarobotics.commands.basic.collect.SetBeltVoltageWithTimeout;
import org.rivierarobotics.commands.basic.collect.SetIntakeVoltage;
import org.rivierarobotics.commands.basic.shoot.SetFloppaPosition;
import org.rivierarobotics.subsystems.intake.IntakeBelt;
import org.rivierarobotics.subsystems.intake.IntakeRollers;
import org.rivierarobotics.subsystems.intake.IntakeSensors;

public class CollectBalls extends SequentialCommandGroup {
    private static final double COLLECT_VOLTAGE = -9;
    private static final double INTAKE_VOLTAGE = 12;
    private static final double MINIWHEEL_VOLTAGE = 7;

    public CollectBalls() {
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        new WaitUntilCommand(() -> !IntakeSensors.getInstance().canCollect()),
                                        new SetBeltAndMiniwheelVoltage(COLLECT_VOLTAGE, MINIWHEEL_VOLTAGE),
                                        new SetIntakeVoltage(INTAKE_VOLTAGE),
                                        new SetFloppaPosition(0)
                                ),
                                new SetBeltVoltageWithTimeout(-COLLECT_VOLTAGE, 0.2)
                        ),
                        new WaitCommand(0.1),
                        () -> IntakeSensors.getInstance().canCollect())
        );
    }

    @Override
    public void end(boolean interrupted) {
        IntakeBelt.getInstance().setBeltVoltage(0);
        IntakeBelt.getInstance().setMiniWheelMotorVoltage(0);
        IntakeRollers.getInstance().setRollerVoltage(0);
    }
}
