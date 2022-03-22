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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
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
    private boolean firstRun = false;
    private boolean shootBadBall = false;
    boolean isCollect;

    FloppaActuator floppaActuator;
    FloppaFlywheels floppaFlywheels;
    IntakeBelt intakeBelt;
    private double startTime = 0.0;

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
                floppaFlywheels,
                intakeBelt
        );
    }

    public Eject(double floppaPosition, double miniwheelVoltage, boolean isCollect){
        this(floppaPosition, miniwheelVoltage, isCollect, 0);
    }

    @Override
    public void initialize() {
        floppaFlywheels.setFlywheelSpeed(flywheelSpeed);
        isEjectPos = true;
        firstRun = false;
        shootBadBall = false;
    }

    private void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    private boolean timerFinished(double waitTime) {
        return Timer.getFPGATimestamp() - startTime >= waitTime;
    }



    @Override
    public void execute(){
        SmartDashboard.putBoolean("SHOULD DO THINGS", !IntakeSensors.getInstance().isTeamBall() && !isEjectPos);
        SmartDashboard.putBoolean("BAD BALL", shootBadBall);

        if(!IntakeSensors.getInstance().isTeamBall() && !isEjectPos) {
            floppaActuator.setFloppaAngle(1);
            if(isCollect){
                intakeBelt.setMiniWheelMotorVoltage(-miniwheelVoltage);
                floppaFlywheels.setVoltage(8);
            } else {
                shootBadBall = true;
                intakeBelt.setBeltVoltage(0);
                intakeBelt.setMiniWheelMotorVoltage(-miniwheelVoltage);
                floppaFlywheels.setVoltage(0);
                startTimer();
            }
            isEjectPos = true;
            startTimer();
        }
        else if(isEjectPos) {
            if(!timerFinished(0.25) && !firstRun) return;
            if((shootBadBall && !timerFinished(0.5)) && !firstRun) return;

            isEjectPos = false;
            if(!shootBadBall) {
                floppaActuator.setFloppaAngle(floppaPosition);
            } else {
                floppaFlywheels.setFlywheelSpeed(flywheelSpeed);
            }
            intakeBelt.setMiniWheelMotorVoltage(miniwheelVoltage);
            if(isCollect){
                floppaActuator.setFloppaAngle(0);
                floppaFlywheels.setVoltage(0);
            }
            if(!isCollect) {
                intakeBelt.setBeltVoltage(-7);
            } else {
                intakeBelt.setBeltVoltage(CollectBalls.COLLECT_VOLTAGE);
            }
            firstRun = true;
        }
    }
}
