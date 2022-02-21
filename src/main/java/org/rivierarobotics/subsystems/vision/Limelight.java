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

public class Limelight extends SubsystemBase {
    private final PhotonCamera camera;


    private static Limelight limelight;


    public static Limelight getInstance() {
        if (limelight == null) {
            limelight = new Limelight();
        }
        return limelight;
    }

    public Limelight(){
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

    public boolean getDetected(){
        return camera.getLatestResult().hasTargets();
    }

    public double getDistance(){
        return Math.atan(Math.toRadians(getTy() + ShooterConstants.getLLAngle()))*(ShooterConstants.getGoalHeight() - ShooterConstants.getRobotHeight());
    }

    @Override
    public void periodic() {
    }


}
