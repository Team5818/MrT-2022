package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.subsystems.climb.ClimbLocation;
import org.rivierarobotics.subsystems.climb.Pistons;

public class RunClimb extends SequentialCommandGroup {

    public RunClimb() {
        addCommands(
                new SetPistonState(Pistons.MID, true),
                new SetPistonState(Pistons.HIGH, true),
                new ClimbSetPosition(ClimbLocation.MID),
                new SetPistonState(Pistons.MID, false),
                new WaitCommand(1.5),
                new SetPistonState(Pistons.LOW, true),
                new ClimbSetPosition(ClimbLocation.HIGH),
                new SetPistonState(Pistons.HIGH, false),
                new WaitCommand(1.5),
                new SetPistonState(Pistons.MID, true)
        );
    }
}
