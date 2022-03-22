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
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.IntakeBelt;
import org.rivierarobotics.subsystems.intake.IntakeSensors;
import org.rivierarobotics.subsystems.shoot.FloppaActuator;
import org.rivierarobotics.subsystems.shoot.FloppaFlywheels;
import org.rivierarobotics.subsystems.shoot.ShootingTables;
import org.rivierarobotics.subsystems.vision.Limelight;

public class EjectShoot extends CommandBase {

    private double floppaPosition;
    private double flywheelSpeed;
    private final double miniwheelVoltage;
    private boolean isEjectPos = true;
    private boolean firstRun = false;
    private boolean useLimelight = false;
    private final double beltVoltage;

    FloppaActuator floppaActuator;
    FloppaFlywheels floppaFlywheels;
    IntakeBelt intakeBelt;
    private double startTime = 0.0;

    public EjectShoot(double floppaPosition, double miniwheelVoltage, double flywheelSpeed, double beltVoltage) {
        floppaActuator = FloppaActuator.getInstance();
        floppaFlywheels = FloppaFlywheels.getInstance();
        intakeBelt = IntakeBelt.getInstance();

        this.floppaPosition = floppaPosition;
        this.miniwheelVoltage = miniwheelVoltage;
        this.flywheelSpeed = flywheelSpeed;
        this.beltVoltage = beltVoltage;

        addRequirements(
                floppaActuator,
                floppaFlywheels,
                intakeBelt
        );
    }

    public EjectShoot(double miniwheelVoltage, double beltVoltage) {
        this(0,miniwheelVoltage, 0, beltVoltage);
        useLimelight = true;
    }

    @Override
    public void initialize() {
        isEjectPos = true;
        firstRun = false;

        if(useLimelight) {
            this.flywheelSpeed = ShootingTables.getFloppaSpeedTable().getValue(Limelight.getInstance().getDistance());
            this.floppaPosition = ShootingTables.getFloppaAngleTable().getValue(Limelight.getInstance().getDistance());
        }

        this.floppaActuator.setFloppaAngle(this.floppaPosition);
        this.floppaFlywheels.setFlywheelSpeed(this.flywheelSpeed);
    }

    private void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    private boolean timerFinished(double waitTime) {
        return Timer.getFPGATimestamp() - startTime >= waitTime;
    }

    @Override
    public void execute() {
        if (!IntakeSensors.getInstance().isTeamBall() && !isEjectPos) {
            floppaActuator.setFloppaAngle(0);
            intakeBelt.setBeltVoltage(0);
            intakeBelt.setMiniWheelMotorVoltage(-miniwheelVoltage);
            floppaFlywheels.setVoltage(0);
            isEjectPos = true;
            startTimer();
        } else if (isEjectPos) {
            if (!timerFinished(0.5) && !firstRun) return;
            isEjectPos = false;
            floppaFlywheels.setFlywheelSpeed(flywheelSpeed);
            intakeBelt.setMiniWheelMotorVoltage(miniwheelVoltage);
            intakeBelt.setBeltVoltage(beltVoltage);
            firstRun = true;
        }
    }
}
