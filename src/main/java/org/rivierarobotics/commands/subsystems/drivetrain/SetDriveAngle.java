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

package org.rivierarobotics.commands.subsystems.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.ml.MLCore;


public class SetDriveAngle extends CommandBase {
    private final DriveTrain dt;
    private final Gyro gyro;
    private final double angle;

    public SetDriveAngle(double angle) {
        this.dt = DriveTrain.getInstance();
        this.gyro = Gyro.getInstance();
        this.angle = angle;
        addRequirements(this.dt);
    }

    @Override
    public void initialize() {
        dt.setTargetRotationAngle(angle);
    }

    public static double MIN_ROT = 0.0;
    public static double TURN_SPEED = 0.15;
    public static double MAX_SPEED = 5;

    private double getRotationSpeed() {
        if (MathUtil.isWithinTolerance(Gyro.getInstance().getRotation2d().getDegrees(), dt.getTargetRotationAngle(), 2)) {
            return 0.0;
        }
        double vel = (TURN_SPEED * (dt.getTargetRotationAngle() - Gyro.getInstance().getRotation2d().getDegrees()));
        if (Math.abs(vel) < MIN_ROT) return Math.signum(vel) * MIN_ROT;
        return Math.signum(vel) * Math.min(Math.abs(vel), MAX_SPEED);
    }

    @Override
    public void execute() {
        dt.drive(0, 0, getRotationSpeed(), true);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("DRIVE ANGLE",MathUtil.isWithinTolerance(dt.getTargetRotationAngle(), gyro.getRotation2d().getDegrees(), 1) );
        return MathUtil.isWithinTolerance(dt.getTargetRotationAngle(), gyro.getRotation2d().getDegrees(), 1);
    }

    @Override
    public void end(boolean interrupted) {
        dt.drive(0, 0, 0, false);
    }
}
