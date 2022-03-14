/*
 * This file is part of Placeholder-2022, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.subsystems.floppas.SetFloppaPosition;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.subsystems.shoot.Floppas;

public class Shoot extends SequentialCommandGroup {

    public Shoot(){
        addRequirements(Floppas.getInstance(), Intake.getInstance());
        addCommands(
                new InstantCommand(() -> Floppas.getInstance().setSpeed(Floppas.getInstance().getTargetV())),
                new WaitCommand(0.35),
                new InstantCommand(() -> Intake.getInstance().setVoltages(-11, 0)),
                new WaitCommand(2),
                new InstantCommand(() -> Floppas.getInstance().setSpeed(0)),
                new InstantCommand(() -> Intake.getInstance().setVoltages(0,0))
        );
    }

    public Shoot(boolean useFloppas) {
        addRequirements(Floppas.getInstance(), Intake.getInstance());
        addCommands(
                new InstantCommand(() -> Floppas.getInstance().setSpeed(Floppas.getInstance().getTargetV())).andThen(new WaitCommand(2)),
                new InstantCommand(() -> Intake.getInstance().setVoltages(-11, 0)),
                new WaitCommand(0.2),
                new InstantCommand( ()-> Intake.getInstance().setVoltages(0,0)),
                new WaitCommand(0.2),
                new InstantCommand(()-> Intake.getInstance().setVoltages(-11,0)),
                new WaitCommand(0.5),
                new InstantCommand(() -> Floppas.getInstance().setSpeed(0)),
                new InstantCommand(() -> Intake.getInstance().setVoltages(0,0))
        );
    }

    public Shoot(Floppas.ShooterLocations locations){
        this(locations.flyWheelSpeed, locations.floppaAngle);
    }

    public Shoot(double speed, double flywheelAngle) {
        addRequirements(Floppas.getInstance(), Intake.getInstance());
        addCommands(
                new SetFloppaPosition(flywheelAngle).withTimeout(2),
                new InstantCommand(() -> Floppas.getInstance().setSpeed(speed))
                        .until(() -> Floppas.getInstance().getLeftSpeed() >= speed)
                        .withTimeout(1),
                new WaitCommand(0.4),
                new InstantCommand(() -> Intake.getInstance().setVoltages(-11, 0)),
                new WaitCommand(0.2),
                new InstantCommand( ()-> Intake.getInstance().setVoltages(0,0)),
                new WaitCommand(0.2),
                new InstantCommand(()-> Intake.getInstance().setVoltages(-11,0)),
                new WaitCommand(0.5),
                new InstantCommand(() -> Floppas.getInstance().setSpeed(0)),
                new InstantCommand(() -> Intake.getInstance().setVoltages(0,0))
        );
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().setVoltages(0,0);
        Floppas.getInstance().setSpeed(0);
        Floppas.getInstance().setShooterVoltage(0);
    }
}
