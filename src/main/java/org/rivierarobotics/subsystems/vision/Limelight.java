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

package org.rivierarobotics.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;

// do not make this a subsystem please thank you
public class Limelight {
    private static Limelight INSTANCE;

    public static Limelight getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Limelight();
        }
        return INSTANCE;
    }

    private static final double LL_ANGLE = 30; // deg
    private static final double ROBOT_HEIGHT = 1.092; // m;
    private static final double GOAL_HEIGHT = 2.6416; // m
    private static final double LL_OFFSET = 0.2286;
    private double targetX = 8.22;
    private double targetY = 4.11;
    private double storedAngle;

    private final PhotonCamera camera;

    private final Pose2d limelightTargetPose = new Pose2d();

    public Limelight() {
        this.camera = new PhotonCamera("gloworm");
    }

    public double getTy() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.targets.get(0).getPitch();
        } else {
            return 0;
        }
    }

    public double getTx() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.targets.get(0).getYaw();
        } else {
            return 0;
        }
    }

    public boolean getDetected() {
        return camera.getLatestResult().hasTargets();
    }

    public double getDistance() {
        return PhotonUtils.calculateDistanceToTargetMeters(ROBOT_HEIGHT, GOAL_HEIGHT, Math.toRadians(LL_ANGLE), Math.toRadians(getTy()));
    }

    public double getAdjustedDistance() {
        var dist = getDistance();
        return Math.sqrt(Math.pow(dist, 2) + Math.pow(LL_OFFSET, 2) - 2 * dist * LL_OFFSET * Math.cos(Math.toRadians(90 + getTx())));
    }

    // returns new Tx in Radians
    public double getAdjustedTx() {
        var adj = getAdjustedDistance();
        var tx = getTx();
        var dist = getDistance();
        var cutoff = Math.sqrt(dist * dist - LL_OFFSET * LL_OFFSET);
        var txp = 90.0 - Math.toDegrees(Math.asin((Math.sin(Math.toRadians(90.0 + tx)) / (adj * dist)) % 1.0));
        var fin = adj < cutoff ? -1 * txp : txp;
        if (Math.abs(getDistance() - getAdjustedDistance()) < 45) {
            this.storedAngle = fin;
        }
        return storedAngle;
    }

    public double getShootingAssistAngle() {
        var robotX = DriveTrain.getInstance().getPoseEstimator().getRobotPose().getX();
        var robotY = DriveTrain.getInstance().getPoseEstimator().getRobotPose().getY();

        var targetAngle = -Math.atan((targetX - robotX) / (targetY - robotY));
        return Math.toDegrees(targetAngle);
    }

    public Pose2d getLLAbsPose() {
        double xpose = Math.cos(Gyro.getInstance().getRotation2d().getRadians() + Math.toRadians(getTx() - 90)) * getDistance() + targetX;
        double ypose = Math.sin(Gyro.getInstance().getRotation2d().getRadians() + Math.toRadians(getTx() - 90)) * getDistance() + targetY;
        return new Pose2d(new Translation2d(xpose, ypose), Gyro.getInstance().getRotation2d());
    }
}
