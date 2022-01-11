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

package org.rivierarobotics.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class Gyro {
    private static Gyro gyro;

    public static Gyro getInstance() {
        if (gyro == null) {
            gyro = new Gyro();
        }
        return gyro;
    }

    private final AHRS navX;

    private Gyro() {
        this.navX = new AHRS(SPI.Port.kMXP);
    }

    public double getAngle() {
        return navX.getAngle();
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(Math.toRadians(getAngle()));
    }

    public double getRate() {
        return Math.toRadians(navX.getRate());
    }

    public void setAngleAdjustment(double angle) {
        navX.reset();
        navX.setAngleAdjustment(angle);
    }

    public void resetGyro() {
        navX.reset();
    }
}