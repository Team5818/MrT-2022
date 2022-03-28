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

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.IntakeBelt;
import org.rivierarobotics.subsystems.intake.IntakeSensors;
import org.rivierarobotics.subsystems.shoot.FloppaActuator;
import org.rivierarobotics.subsystems.shoot.FloppaFlywheels;
import org.rivierarobotics.subsystems.shoot.ShootingTables;
import org.rivierarobotics.subsystems.vision.Limelight;
import org.rivierarobotics.util.RRTimer;

public class EjectShoot extends CommandBase {
    private double floppaPosition;
    private double flywheelSpeed;
    private final double miniwheelVoltage;
    private boolean isEjectPos = true;
    private boolean firstRun = false;
    private boolean useLimelight = false;
    private final double beltVoltage;

    private final FloppaActuator floppaActuator;
    private final FloppaFlywheels floppaFlywheels;
    private final IntakeBelt intakeBelt;
    private RRTimer rrTimer;

    public EjectShoot(double floppaPosition, double miniwheelVoltage, double flywheelSpeed, double beltVoltage) {
        this.floppaActuator = FloppaActuator.getInstance();
        this.floppaFlywheels = FloppaFlywheels.getInstance();
        this.intakeBelt = IntakeBelt.getInstance();

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
        this(0, miniwheelVoltage, 0, beltVoltage);
        this.useLimelight = true;
    }

    @Override
    public void initialize() {
        this.isEjectPos = true;
        this.firstRun = false;
        this.rrTimer = new RRTimer(0.5);
        if (useLimelight) {
            this.flywheelSpeed = ShootingTables.getFloppaSpeedTable().getValue(Limelight.getInstance().getDistance());
            this.floppaPosition = ShootingTables.getFloppaAngleTable().getValue(Limelight.getInstance().getDistance());
        }

        this.floppaActuator.setFloppaAngle(this.floppaPosition);
        this.floppaFlywheels.setFlywheelSpeed(this.flywheelSpeed);
    }

    @Override
    public void execute() {
        if (!IntakeSensors.getInstance().isTeamBall() && !isEjectPos) {
            floppaActuator.setFloppaAngle(0);
            intakeBelt.setBeltVoltage(0);
            intakeBelt.setMiniWheelMotorVoltage(-miniwheelVoltage);
            floppaFlywheels.setVoltage(0);
            this.isEjectPos = true;
            rrTimer.reset();
        } else if (isEjectPos) {
            if (!rrTimer.finished() && !firstRun) {
                return;
            }
            this.isEjectPos = false;
            floppaFlywheels.setFlywheelSpeed(flywheelSpeed);
            intakeBelt.setMiniWheelMotorVoltage(miniwheelVoltage);
            intakeBelt.setBeltVoltage(beltVoltage);
            this.firstRun = true;
        }
    }
}
