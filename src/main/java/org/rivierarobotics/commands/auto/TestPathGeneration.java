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

package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.apache.commons.math3.analysis.function.Log;
import org.rivierarobotics.lib.shuffleboard.RobotShuffleboard;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.aifield.FieldMesh;

public class TestPathGeneration extends CommandBase {
    private final DriveTrain driveTrain;
    private final FieldMesh aiFieldMesh;
    private final double relativeX;
    private final double relativeY;

    public TestPathGeneration(double relativeX, double relativeY) {
        this.driveTrain = DriveTrain.getInstance();
        this.aiFieldMesh = FieldMesh.getInstance();
        this.relativeX = relativeX;
        this.relativeY = relativeY;
    }

    @Override
    public void initialize() {
        var currentX = driveTrain.getRobotPose().getX();
        var currentY = driveTrain.getRobotPose().getY();
        var trajectory = aiFieldMesh.getTrajectory(currentX, currentY, currentX + relativeX, currentY + relativeY, true, 0, driveTrain.getSwerveDriveKinematics());

        Logging.robotShuffleboard.getTab("Drive").setEntry("target position X", currentX + relativeX);
        Logging.robotShuffleboard.getTab("Drive").setEntry("target position Y", currentY + relativeY);
        Logging.aiFieldDisplay.updatePath(trajectory);
        if (trajectory != null) {
            driveTrain.drivePath(trajectory);
        }
    }

    @Override
    public boolean isFinished() {
        return !driveTrain.followHolonomicController();
    }
}