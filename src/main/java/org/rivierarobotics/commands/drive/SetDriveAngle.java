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

package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;


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
    public void execute() {
        dt.drive(0, 0, Math.signum(angle) * DriveTrain.MAX_ANGULAR_SPEED, false);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(gyro.getAngle(), angle, 1);
    }
}
