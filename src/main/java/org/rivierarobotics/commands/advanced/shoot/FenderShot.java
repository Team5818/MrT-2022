package org.rivierarobotics.commands.advanced.shoot;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.basic.collect.SetBeltVoltage;
import org.rivierarobotics.commands.basic.collect.SetMiniwheelVoltage;
import org.rivierarobotics.commands.basic.shoot.SetFloppaPosition;
import org.rivierarobotics.commands.basic.shoot.SetFlywheelSpeed;
import org.rivierarobotics.subsystems.shoot.ShooterLocations;

public class FenderShot extends SequentialCommandGroup {

    public FenderShot(){
        super(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2),
                                        new WaitCommand(0.5).andThen(new SetBeltVoltage(ShootAll.SHOOT_BELT_VOLTAGE)).andThen(new SetMiniwheelVoltage(ShootAll.SHOOT_MINIWHEEL_VOLTAGE)),
                                        new SetFlywheelSpeed(ShooterLocations.FENDER.flyWheelSpeed)
                                )
                        ),
                        new SetFloppaPosition(ShooterLocations.FENDER).withTimeout(1)
                ),
                new WaitCommand(1),
                new SetFlywheelSpeed(0),
                new SetBeltVoltage(0),
                new SetMiniwheelVoltage(0)
        );
    }
}
