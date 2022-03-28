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

package org.rivierarobotics.commands.advanced.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Limelight;
import org.rivierarobotics.util.Gyro;


public class AngulationToTargetBasedOffOfPose extends CommandBase {
    private final DriveTrain dt;
    private final Gyro gyro;

    public AngulationToTargetBasedOffOfPose() {
        this.dt = DriveTrain.getInstance();
        this.gyro = Gyro.getInstance();
        addRequirements(this.dt);
    }

    @Override
    public void initialize() {
        dt.setTargetRotationAngle(Limelight.getInstance().getShootingAssistAngle());
    }

    @Override
    public void execute() {
        dt.drive(0, 0, dt.getRotationSpeed(), true);
    }

    @Override
    public boolean isFinished() {
        return !MathUtil.isWithinTolerance(gyro.getRotation2d().getDegrees(), dt.getTargetRotationAngle(), 1);
    }

    @Override
    public void end(boolean interrupted) {
        dt.drive(0, 0, 0, false);
    }
}
