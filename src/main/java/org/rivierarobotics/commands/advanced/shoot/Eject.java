/*
 * This file is part of MrT-2022, licensed under the GNU General Public License (GPLv3).
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

//TODO: Delete this whole class
public class Eject extends CommandBase {
    private final double floppaPosition;
    private final double miniwheelVoltage;
    private final double flywheelSpeed;
    private final boolean isCollect;
    private boolean isEjectPos = true;
    private boolean firstRun = false;
    private boolean shootBadBall = false;

    private final FloppaActuator floppaActuator;
    private final FloppaFlywheels floppaFlywheels;
    private final IntakeBelt intakeBelt;
    private double startTime = 0.0;

    public Eject(double floppaPosition, double miniwheelVoltage, boolean isCollect, double flywheelSpeed) {
        this.floppaActuator = FloppaActuator.getInstance();
        this.floppaFlywheels = FloppaFlywheels.getInstance();
        this.intakeBelt = IntakeBelt.getInstance();

        this.floppaPosition = floppaPosition;
        this.miniwheelVoltage = miniwheelVoltage;
        this.isCollect = isCollect;
        this.flywheelSpeed = flywheelSpeed;

        addRequirements(
            this.floppaActuator,
            this.floppaFlywheels,
            this.intakeBelt
        );
    }

    public Eject(double floppaPosition, double miniwheelVoltage, boolean isCollect) {
        this(floppaPosition, miniwheelVoltage, isCollect, 0);
    }

    @Override
    public void initialize() {
        this.floppaFlywheels.setFlywheelSpeed(flywheelSpeed);
        this.isEjectPos = true;
        this.firstRun = false;
        this.shootBadBall = false;
    }

    //TODO startTimer(), timerFinished(), and startTime are copied across:
    // Eject, EjectCollect, and EjectShoot
    // please pull this out into a class that you can use, i.e. RRTimer or something
    // alternatively you could have that class extend CommandBase and you could
    // extend some new TimedCommand or something.

    private void startTimer() {
        this.startTime = Timer.getFPGATimestamp();
    }

    private boolean timerFinished(double waitTime) {
        return Timer.getFPGATimestamp() - startTime >= waitTime;
    }

    @Override
    public void execute() {
        //TODO remove SmartDashboard calls on prod/master
        SmartDashboard.putBoolean("SHOULD DO THINGS", !IntakeSensors.getInstance().isTeamBall() && !isEjectPos);
        SmartDashboard.putBoolean("BAD BALL", shootBadBall);

        //TODO I feel like this should not be this complicated but idk, cleanup if possible
        if (!IntakeSensors.getInstance().isTeamBall() && !isEjectPos) {
            //TODO what is this magic 1 constant? pull to const field?
            floppaActuator.setFloppaAngle(1);
            if (isCollect) {
                intakeBelt.setMiniWheelMotorVoltage(-miniwheelVoltage);
                //TODO what is this magic 8 constant? pull to const field?
                floppaFlywheels.setVoltage(8);
            } else {
                this.shootBadBall = true;
                intakeBelt.setBeltVoltage(0);
                intakeBelt.setMiniWheelMotorVoltage(-miniwheelVoltage);
                floppaFlywheels.setVoltage(0);
                startTimer();
            }
            this.isEjectPos = true;
            startTimer();
        } else if (isEjectPos) {
            //TODO simplify this logic
            if ((!timerFinished(0.25) && !firstRun) || ((shootBadBall && !timerFinished(0.5)) && !firstRun)) {
                return;
            }

            this.isEjectPos = false;
            if (!shootBadBall) {
                floppaActuator.setFloppaAngle(floppaPosition);
            } else {
                floppaFlywheels.setFlywheelSpeed(flywheelSpeed);
            }
            intakeBelt.setMiniWheelMotorVoltage(miniwheelVoltage);
            if (isCollect) {
                floppaActuator.setFloppaAngle(0);
                floppaFlywheels.setVoltage(0);
            }
            //TODO what is this magic -7 constant? pull to const field?
            intakeBelt.setBeltVoltage(isCollect ? CollectBalls.COLLECT_VOLTAGE : -7);
            this.firstRun = true;
        }
    }
}
