package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.collect.CollectToggle;
import org.rivierarobotics.commands.drive.SetWheelbaseAngle;

public class twoBallAuto extends SequentialCommandGroup {

    public twoBallAuto() {
        addCommands(
                //Shoot command
                new ParallelDeadlineGroup( new DrivePath("2BallTest1"), new CollectToggle(false, true, true)),
                new SetWheelbaseAngle(0)
                //Shoot command
        );
    }
}
