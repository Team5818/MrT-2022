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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.aifield.FieldMesh;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.swerve.TrajectoryFollower;

import java.util.Comparator;

public class DriveToClosest extends SequentialCommandGroup {
    private final DriveTrain driveTrain;
    private final Gyro gyro;
    private final FieldMesh aiFieldMesh;
    private TrajectoryFollower trajectoryFollower;


    public DriveToClosest() {
        this.driveTrain = DriveTrain.getInstance();
        this.aiFieldMesh = FieldMesh.getInstance();
        this.gyro = Gyro.getInstance();
    }

    @Override
    public void initialize() {
        gyro.resetGyro();

        var currentX = driveTrain.getPoseEstimator().getRobotPose().getX();
        var currentY = driveTrain.getPoseEstimator().getRobotPose().getY();


        MLCore core = MLCore.getInstance();
        var ballColor = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? "blue" : "red";
        var balls = core.getDetectedObjects().get(ballColor);
        if(balls == null || balls.isEmpty()) {
            return;
        }

        balls.sort(Comparator.comparingDouble(a -> a.relativeLocationDistance));
        var ball = balls.get(0);
        var targetX = currentX + Math.cos(Gyro.getInstance().getRotation2d().getRadians() + Math.toRadians(ball.ty)) * ball.relativeLocationDistance;
        var targety = currentY + Math.sin(Gyro.getInstance().getRotation2d().getRadians() + Math.toRadians(ball.ty)) * ball.relativeLocationDistance;

        var trajectory = aiFieldMesh.getTrajectory(currentX, currentY, targetX,  targety, true, 0, driveTrain.getSwerveDriveKinematics());
        Logging.aiFieldDisplay.updatePath(trajectory);

        trajectoryFollower = new TrajectoryFollower(trajectory, false, Gyro.getInstance(), driveTrain);
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
