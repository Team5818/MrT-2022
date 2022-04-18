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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.ml.MLCore;

import java.util.Comparator;

public class RotateBall extends CommandBase {
    private final DriveTrain driveTrain;
    public RotateBall() {
        this.driveTrain = DriveTrain.getInstance();
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        MLCore core = MLCore.getInstance();
        var ballColor = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? "blue" : "red";

        var balls = core.getDetectedObjects().get(ballColor);
        if (balls == null || balls.isEmpty()) {
            return;
        }

        balls.sort(Comparator.comparingDouble(a -> a.relativeLocationDistance));

        driveTrain.setTargetRotationAngle(Gyro.getInstance().getRotation2d().getDegrees() + balls.get(0).ty);
        driveTrain.drive(0,0,driveTrain.getRotationSpeed(), true);
    }

    @Override
    public boolean isFinished() {
        return driveTrain.getRotationSpeed() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(0,0,0,true);
    }
}
