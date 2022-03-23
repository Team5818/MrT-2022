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

package org.rivierarobotics.commands.advanced.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.aifield.FieldMesh;
import org.rivierarobotics.util.swerve.TrajectoryFollower;

public class PathGeneration extends CommandBase {
    private final DriveTrain driveTrain;
    private final FieldMesh aiFieldMesh;
    private final double relativeX;
    private final double relativeY;
    private TrajectoryFollower trajectoryFollower;

    public PathGeneration(double relativeX, double relativeY) {
        this.driveTrain = DriveTrain.getInstance();
        this.aiFieldMesh = FieldMesh.getInstance();
        this.relativeX = relativeX;
        this.relativeY = relativeY;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        //TODO this is the same value twice right? driveTrain.getPoseEstimator().getRobotPose()?
        // so save to var and use twice instead of making an extra two calls
        var currentX = driveTrain.getPoseEstimator().getRobotPose().getX();
        var currentY = driveTrain.getPoseEstimator().getRobotPose().getY();
        var trajectory = aiFieldMesh.getTrajectory(currentX, currentY, currentX + relativeX,
            currentY + relativeY, true, 0, driveTrain.getSwerveDriveKinematics());
        this.trajectoryFollower = new TrajectoryFollower(trajectory, false, Gyro.getInstance(), driveTrain);
    }

    @Override
    public void execute() {
        trajectoryFollower.followController();
    }

    @Override
    public boolean isFinished() {
        return trajectoryFollower.isFinished();
    }
}