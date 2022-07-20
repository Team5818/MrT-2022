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

package org.rivierarobotics.commands.basic.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.shoot.FloppaActuator;
import org.rivierarobotics.subsystems.shoot.FloppaFlywheels;
import org.rivierarobotics.subsystems.shoot.ShootingTables;
import org.rivierarobotics.subsystems.vision.Limelight;

public class SetFloppaLimelight extends CommandBase {
    private final FloppaActuator floppasActuator;
    private final FloppaFlywheels floppaFlywheels;
    private final boolean isLimelight;
    private final double angle;
    private final double speed;

    public SetFloppaLimelight(boolean isLimelight, double angle, double speed) {
        this.floppasActuator = FloppaActuator.getInstance();
        this.floppaFlywheels = FloppaFlywheels.getInstance();
        this.isLimelight = isLimelight;
        this.angle = angle;
        this.speed = speed;
        addRequirements(floppasActuator, floppaFlywheels);
    }

    public SetFloppaLimelight(boolean isLimelight) {
        this(isLimelight, 0, 0);
    }

    @Override
    public void initialize() {
        if (isLimelight) {
            double dist = Limelight.getInstance().getDistance();
            this.floppasActuator.setFloppaAngle(ShootingTables.createFloppaAngleTable().getValue(dist));
            this.floppaFlywheels.setFlywheelSpeed(ShootingTables.createFloppaSpeedTable().getValue(dist));
        } else {
            this.floppasActuator.setFloppaAngle(angle);
            this.floppaFlywheels.setFlywheelSpeed(speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        floppasActuator.setVoltage(0);
        floppaFlywheels.setTargetVelocity(0);
    }
}
