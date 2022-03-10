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

package org.rivierarobotics.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class Limelight extends SubsystemBase {

    public static Limelight getInstance() {
        if (limelight == null) {
            limelight = new Limelight();
        }
        return limelight;
    }

    private static Limelight limelight;
    private static double llAngle = 30;
    private static double robotHeight = 1.092;
    private static double goalHeight = 2.6416;
    private final PhotonCamera camera;
    private static final double llOffset = 0.1;

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
        return PhotonUtils.calculateDistanceToTargetMeters(robotHeight, goalHeight, Math.toRadians(llAngle), Math.toRadians(getTy()));
    }

    @Override
    public void periodic() {
    }

    public double getAdjustedDistance () {
         return Math.sqrt(getDistance() * getDistance() + llOffset * llOffset - getDistance() * llOffset * Math.cos(getTx()));
    }
    public double getAdjustedTx() {
        return Math.acos(getDistance() * Math.cos(getTx()) / getAdjustedDistance());
    }

}
