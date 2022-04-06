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
import org.rivierarobotics.util.RRTimer;

public class EjectCollect extends CommandBase {
    private final double miniwheelVoltage;
    private final double beltVoltage;
    private boolean isEjectPos = true;
    private boolean firstRun = false;
    private RRTimer timer;

    private final FloppaActuator floppaActuator;
    private final IntakeBelt intakeBelt;
    private final FloppaFlywheels floppaFlywheels;

    public EjectCollect(double beltVoltage, double miniwheelVoltage) {
        this.floppaActuator = FloppaActuator.getInstance();
        this.floppaFlywheels = FloppaFlywheels.getInstance();
        this.intakeBelt = IntakeBelt.getInstance();

        this.miniwheelVoltage = miniwheelVoltage;
        this.beltVoltage = beltVoltage;

        addRequirements(
            floppaActuator,
            floppaFlywheels,
            intakeBelt
        );
    }

    @Override
    public void initialize() {
        floppaActuator.setFloppaAngle(0);
        this.isEjectPos = true;
        this.firstRun = false;
        this.timer = new RRTimer(0.25);
    }

    @Override
    public void execute() {
        if (!IntakeSensors.getInstance().isTeamBall() && !isEjectPos) {
            floppaActuator.setFloppaAngle(0);
            intakeBelt.setMiniWheelMotorVoltage(-miniwheelVoltage);
            floppaFlywheels.setVoltage(11);
            this.isEjectPos = true;
            timer.reset();
        } else if (isEjectPos) {
            if (!timer.finished() && !firstRun) {
                return;
            }
            this.isEjectPos = false;
            floppaFlywheels.setVoltage(0);
            intakeBelt.setMiniWheelMotorVoltage(miniwheelVoltage);
            intakeBelt.setBeltVoltage(beltVoltage);
            this.firstRun = true;
        }
    }
}
