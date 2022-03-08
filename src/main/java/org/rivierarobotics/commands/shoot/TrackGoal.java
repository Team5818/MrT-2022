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

package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Floppas;
import org.rivierarobotics.subsystems.vision.Limelight;
import org.rivierarobotics.util.Gyro;

public class TrackGoal extends CommandBase {
    private final Floppas floppas;
    private final DriveTrain drive;
    private final Limelight lime;
    private double storedTx;
    private Gyro gyro;

    public TrackGoal() {
        this.floppas = Floppas.getInstance();
        this.drive = DriveTrain.getInstance();
        this.lime = Limelight.getInstance();
        this.gyro = Gyro.getInstance();
        addRequirements(floppas);
    }

    @Override
    public void execute() {
        if (lime.getDetected()) {
            this.storedTx = lime.getTx();
            SmartDashboard.putNumber("storedtx", storedTx);
            drive.setTargetRotationAngle(gyro.getRotation2d().getDegrees() - storedTx);
        }
    }

}