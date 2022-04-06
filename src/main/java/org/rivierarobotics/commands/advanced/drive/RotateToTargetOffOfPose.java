package org.rivierarobotics.commands.advanced.drive;/*
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


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Limelight;
import org.rivierarobotics.util.Gyro;

public class RotateToTargetOffOfPose extends CommandBase {
    private final DriveTrain dt;
    private final Limelight limelight;

    public RotateToTargetOffOfPose() {
        this.dt = DriveTrain.getInstance();
        this.limelight = Limelight.getInstance();
        addRequirements(this.dt);
    }


    public RotateToTargetOffOfPose(boolean isAUTO) {
        this.dt = DriveTrain.getInstance();
        this.limelight = Limelight.getInstance();
    }

    @Override
    public void initialize() {
        dt.setTargetRotationAngle(limelight.getShootingAssistAngle());
    }



    @Override
    public boolean isFinished() {
        return dt.getRotationSpeed() == 0;
    }

    @Override
    public void execute() {
        dt.setTargetRotationAngle(limelight.getShootingAssistAngle());
        dt.drive(0, 0, dt.getRotationSpeed(), true);
    }

    @Override
    public void end(boolean interrupted) {
        dt.drive(0, 0, 0, false);
    }
}
