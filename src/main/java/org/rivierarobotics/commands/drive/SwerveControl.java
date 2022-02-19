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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.robot.ControlMap;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;

public class SwerveControl extends CommandBase {
    private final DriveTrain driveTrain;
    private final double MaxTurnSpeed = Math.PI * (1.4 / 8) * (360 / (2 * Math.PI));
    private final ProfiledPIDController pidController =
            new ProfiledPIDController(0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(0,0));

    public SwerveControl() {
        this.driveTrain = DriveTrain.getInstance();
        addRequirements(this.driveTrain);
    }

    private double getRotationSpeed() {
        if(MathUtil.isWithinTolerance(Gyro.getInstance().getRotation2d().getDegrees(), driveTrain.getTargetRotationAngle(), 2.5)) {
            return 0.0;
        }
        double vel = (0.025 * (driveTrain.getTargetRotationAngle() - Gyro.getInstance().getRotation2d().getDegrees()));

        if (Math.abs(vel) > MaxTurnSpeed){
            if (vel > 0){
                return MaxTurnSpeed;
            } else {
                return -MaxTurnSpeed;
            }
        } else {
            return vel;
        }
    }

    @Override
    public void initialize() {
        driveTrain.targetRotationAngle = 0;
    }

    @Override
    public void execute() {
        var leftJoystick = ControlMap.DRIVER_LEFT;
        var rightJoystick = ControlMap.DRIVER_RIGHT;
        var xSpeed = MathUtil.fitDeadband(-leftJoystick.getY()) * DriveTrain.MAX_SPEED;
        var ySpeed = MathUtil.fitDeadband(-leftJoystick.getX()) * DriveTrain.MAX_SPEED;

        var rot = -MathUtil.fitDeadband(rightJoystick.getX()) * DriveTrain.MAX_ANGULAR_SPEED;

        //driveTrain.drive(xSpeed, ySpeed, rot, driveTrain.getFieldCentric());



        if(rot == 0) {
            driveTrain.drive(xSpeed, ySpeed, getRotationSpeed(), driveTrain.getFieldCentric());
        } else {
            driveTrain.setTargetRotationAngle(Gyro.getInstance().getRotation2d().getDegrees());
            driveTrain.drive(xSpeed, ySpeed, rot, driveTrain.getFieldCentric());
        }
    }
}
