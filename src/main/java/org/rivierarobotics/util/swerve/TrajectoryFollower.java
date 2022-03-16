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

package org.rivierarobotics.util.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;

import java.io.IOException;
import java.nio.file.Path;

public class TrajectoryFollower {
    public static Trajectory getTrajectoryFromPathweaver(String path) {
        String trajectoryJSON = "paths/" + path + ".wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        try {
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Pathweaver path not found", e.getStackTrace());
        }
        return null;
    }

    private final HolonomicDriveController holonomicDriveController;
    private final PathPlannerTrajectory pathPlannerTrajectory;
    private final double startTime;
    private final PoseEstimator estimator;
    private final Trajectory trajectory;
    private final Gyro gyro;
    private final DriveTrain driveTrain;

    public TrajectoryFollower(Trajectory trajectory, boolean resetPose, Gyro gyro, DriveTrain driveTrain) {
        this(trajectory, null, resetPose, gyro, driveTrain);
    }

    public TrajectoryFollower(PathPlannerTrajectory trajectory, boolean resetPose, Gyro gyro, DriveTrain driveTrain) {
        this(null, trajectory, resetPose, gyro, driveTrain);
    }

    private TrajectoryFollower(Trajectory trajectory, PathPlannerTrajectory pathPlannerTrajectory,
        boolean resetPose, Gyro gyro, DriveTrain driveTrain) {
        this.holonomicDriveController = driveTrain.getHolonomicDriveController();
        this.pathPlannerTrajectory = pathPlannerTrajectory;
        this.startTime = Timer.getFPGATimestamp();
        this.gyro = gyro;
        this.driveTrain = driveTrain;
        this.estimator = driveTrain.getPoseEstimator();
        this.trajectory = trajectory;
        if (trajectory == null && pathPlannerTrajectory == null) {
            DriverStation.reportError("Trying to drive null path", false);
            return;
        }
        if (resetPose) {
            estimator.resetPose(getInitialPose());
        }
    }

    private Pose2d getInitialPose() {
        if (trajectory != null) {
            return new Pose2d(trajectory.getInitialPose().getTranslation(), gyro.getRotation2d());
        } else {
            return new Pose2d(pathPlannerTrajectory.getInitialPose().getTranslation(), gyro.getRotation2d());
        }
    }

    public boolean isFinished() {
        return (trajectory == null && pathPlannerTrajectory == null) || (Timer.getFPGATimestamp() - this.startTime > this.trajectory.getTotalTimeSeconds());
    }

    /**
     * Call this method periodically to follow a trajectory.
     */
    public void followController() {
        if (trajectory == null && pathPlannerTrajectory == null) {
            return;
        }

        double timePassed = Timer.getFPGATimestamp() - this.startTime;

        var controls = trajectory == null ? followPathPlannerTrajectory(timePassed) : followTrajectory(timePassed);
        driveTrain.drive(controls.vxMetersPerSecond, controls.vyMetersPerSecond, 0, true);
    }

    private ChassisSpeeds followPathPlannerTrajectory(double intTime) {
        var plannerState = (PathPlannerTrajectory.PathPlannerState) pathPlannerTrajectory.sample(intTime);
        return holonomicDriveController.calculate(
                estimator.getRobotPose(),
                plannerState,
                plannerState.holonomicRotation
        );
    }

    private ChassisSpeeds followTrajectory(double intTime) {
        return holonomicDriveController.calculate(
                estimator.getRobotPose(),
                trajectory.sample(0),
                estimator.getRobotPose().getRotation()
        );
    }
}
