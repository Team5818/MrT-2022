package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.collect.CollectToggle;

public class TestCollectAuto extends SequentialCommandGroup {

    public TestCollectAuto() {
        addCommands(
                // deploy intake
                new ParallelDeadlineGroup(new CollectToggle(false, true, true), new DrivePath("straight")).deadlineWith(new WaitCommand(5))
                // retract intake
        );
    }
}
