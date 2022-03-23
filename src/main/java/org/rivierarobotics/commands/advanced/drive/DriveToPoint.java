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
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.aifield.FieldMesh;
import org.rivierarobotics.util.swerve.TrajectoryFollower;

public class DriveToPoint extends CommandBase {
    private final DriveTrain driveTrain;
    private final Gyro gyro;
    private final FieldMesh aiFieldMesh;
    private final double targetY;
    private final double targetX;
    private final boolean shouldStop;
    private final double initialVelocity;
    private TrajectoryFollower trajectoryFollower;

    public DriveToPoint(double targetX, double targetY, boolean shouldStop, double initialVelocity) {
        this.driveTrain = DriveTrain.getInstance();
        this.aiFieldMesh = FieldMesh.getInstance();
        this.gyro = Gyro.getInstance();
        this.targetY = targetY;
        this.targetX = targetX;
        this.shouldStop = shouldStop;
        this.initialVelocity = initialVelocity;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        try {
            var dtPose = driveTrain.getPoseEstimator().getRobotPose();
            var trajectory = aiFieldMesh.getTrajectory(dtPose.getX(), dtPose.getY(), targetX, targetY, shouldStop, initialVelocity, driveTrain.getSwerveDriveKinematics());
            Logging.aiFieldDisplay.updatePath(trajectory);
            this.trajectoryFollower = new TrajectoryFollower(trajectory, false, gyro, driveTrain);
        } catch (Exception e) {
            //invalid trajectory
        }

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
