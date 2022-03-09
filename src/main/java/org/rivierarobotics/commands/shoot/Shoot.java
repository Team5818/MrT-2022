package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.collect.CollectToggle;

public class Shoot extends SequentialCommandGroup {

    public Shoot(double speed){
        addCommands(
                new SetFlywheelSpeed(speed),
                new WaitCommand(0.5),
                new CollectToggle(false, true,false),
                new WaitCommand(0.5),
                new SetFlywheelSpeed(0)
        );
    }


}
