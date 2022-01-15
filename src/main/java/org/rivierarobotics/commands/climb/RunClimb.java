package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.climb.Piston;
import org.rivierarobotics.subsystems.climb.Switch;

public class RunClimb extends CommandBase {
    private SequentialCommandGroup commands;

    public RunClimb() {
        this.commands = new SequentialCommandGroup(
                new SetPistonState(Piston.getInstanceMid(), true),
                new SetPistonState(Piston.getInstanceHigh(), true),
                new ClimbSetPosition(Switch.getInstanceMid()),
                new SetPistonState(Piston.getInstanceMid(), false),
                new SetPistonState(Piston.getInstanceLow(), true),
                new ClimbSetPosition(Switch.getInstanceHigh()),
                new SetPistonState(Piston.getInstanceHigh(), false),
                new SetPistonState(Piston.getInstanceMid(), true)

        );
    }

    @Override
    public void execute() {
        commands.execute();
    }

    @Override
    public boolean isFinished() {
        return commands.isFinished();
    }
}
