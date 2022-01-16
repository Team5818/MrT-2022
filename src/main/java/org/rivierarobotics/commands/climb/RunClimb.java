package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.subsystems.climb.ClimbLocation;
import org.rivierarobotics.subsystems.climb.Piston;

public class RunClimb extends SequentialCommandGroup {

    public RunClimb() {
        addCommands(
                new SetPistonState(Piston.getInstance(Piston.Pistons.MID), true),
                new SetPistonState(Piston.getInstance(Piston.Pistons.HIGH), true),
                new ClimbSetPosition(ClimbLocation.MID),
                new SetPistonState(Piston.getInstance(Piston.Pistons.MID), false),
                new WaitCommand(1.5),
                new SetPistonState(Piston.getInstance(Piston.Pistons.LOW), true),
                new ClimbSetPosition(ClimbLocation.HIGH),
                new SetPistonState(Piston.getInstance(Piston.Pistons.HIGH), false),
                new WaitCommand(1.5),
                new SetPistonState(Piston.getInstance(Piston.Pistons.MID), true)
        );
    }
}
