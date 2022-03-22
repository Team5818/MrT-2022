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

package org.rivierarobotics.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.robot.ControlMap;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;

public class SwerveControl extends CommandBase {
    private final DriveTrain driveTrain;
    private final Gyro gyro;

    public SwerveControl() {
        this.driveTrain = DriveTrain.getInstance();
        this.gyro = Gyro.getInstance();
        addRequirements(this.driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.setTargetRotationAngle(gyro.getRotation2d().getDegrees());
    }

    @Override
    public void execute() {
        var leftJoystick = ControlMap.DRIVER_LEFT;
        var rightJoystick = ControlMap.DRIVER_RIGHT;
        var xSpeed = MathUtil.fitDeadband(-leftJoystick.getY()) * DriveTrain.MAX_TURN_SPEED;
        var ySpeed = MathUtil.fitDeadband(-leftJoystick.getX()) * DriveTrain.MAX_TURN_SPEED;

        var rot = MathUtil.fitDeadband(-rightJoystick.getX()) * DriveTrain.MAX_ANGULAR_SPEED;

        if (rot == 0 && driveTrain.useDriverAssist()) {
            driveTrain.drive(xSpeed, ySpeed, driveTrain.getRotationSpeed(), driveTrain.getFieldCentric());
        } else {
            driveTrain.drive(xSpeed, ySpeed, rot, driveTrain.getFieldCentric());
            driveTrain.setTargetRotationAngle(gyro.getRotation2d().getDegrees());
        }
    }
}
