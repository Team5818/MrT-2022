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

package org.rivierarobotics.commands.advanced.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.basic.collect.SetBeltVoltage;
import org.rivierarobotics.commands.basic.collect.SetMiniwheelVoltage;
import org.rivierarobotics.commands.basic.shoot.FlywheelTest;
import org.rivierarobotics.commands.basic.shoot.SetFloppaPosition;
import org.rivierarobotics.commands.basic.shoot.SetFlywheelSpeed;
import org.rivierarobotics.subsystems.intake.IntakeBelt;
import org.rivierarobotics.subsystems.intake.IntakeSensors;
import org.rivierarobotics.subsystems.shoot.FloppaActuator;
import org.rivierarobotics.subsystems.shoot.FloppaFlywheels;

public class Eject extends CommandBase {

    double floppaPosition, miniwheelVoltage, flywheelSpeed;
    private boolean isEjectPos = true;
    boolean isCollect;

    FloppaActuator floppaActuator;
    FloppaFlywheels floppaFlywheels;
    IntakeBelt intakeBelt;

    public Eject(double floppaPosition, double miniwheelVoltage, boolean isCollect, double flywheelSpeed){
        floppaActuator = FloppaActuator.getInstance();
        floppaFlywheels = FloppaFlywheels.getInstance();
        intakeBelt = IntakeBelt.getInstance();

        this.floppaPosition = floppaPosition;
        this.miniwheelVoltage = miniwheelVoltage;
        this.isCollect = isCollect;
        this.flywheelSpeed = flywheelSpeed;

        addRequirements(
                floppaActuator,
                floppaFlywheels
        );
    }

    public Eject(double floppaPosition, double miniwheelVoltage, boolean isCollect){
        this(floppaPosition, miniwheelVoltage, isCollect, 0);
    }

    @Override
    public void initialize() {
        floppaFlywheels.setFlywheelSpeed(flywheelSpeed);
    }

    @Override
    public void execute(){
        if(!IntakeSensors.getInstance().isTeamBall() && !isEjectPos){
            floppaActuator.setFloppaAngle(0);
            if(isCollect){
                intakeBelt.setMiniWheelMotorVoltage(-miniwheelVoltage);
                floppaFlywheels.setVoltage(4);
            }
            isEjectPos = true;
        }
        else if(isEjectPos){
            isEjectPos = false;
            floppaActuator.setFloppaAngle(floppaPosition);
            intakeBelt.setMiniWheelMotorVoltage(miniwheelVoltage);
            if(isCollect){
                floppaFlywheels.setVoltage(0);
            }
        }
    }
}
