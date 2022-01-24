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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.robot.ControlMap;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class SwerveControl extends CommandBase {
    private final DriveTrain driveTrain;

    public SwerveControl() {
        this.driveTrain = DriveTrain.getInstance();
        addRequirements(this.driveTrain);
    }

    @Override
    public void execute() {
        var leftJoystick = ControlMap.DRIVER_LEFT;
        var rightJoystick = ControlMap.DRIVER_RIGHT;
        var xSpeed = MathUtil.fitDeadband(-leftJoystick.getY()) * DriveTrain.MAX_SPEED;
        var ySpeed = MathUtil.fitDeadband(-leftJoystick.getX()) * DriveTrain.MAX_SPEED;
        var rot = MathUtil.fitDeadband(rightJoystick.getX()) * DriveTrain.MAX_ANGULAR_SPEED;
        SmartDashboard.putBoolean("joystick running", true);
        SmartDashboard.putNumber("JS1", leftJoystick.getY());
        SmartDashboard.putNumber("JS2", rightJoystick.getX());
        SmartDashboard.putNumber("JS", System.nanoTime());
        driveTrain.drive(xSpeed, ySpeed, rot, true);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("joystick running", false);
    }
}
