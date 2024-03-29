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

package org.rivierarobotics.commands.advanced.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Limelight;
import org.rivierarobotics.util.Gyro;

public class TrackGoal extends CommandBase {
    private static final double TOLERANCE = 3;
    private final DriveTrain drive;
    private final Limelight lime;
    private final Gyro gyro;
    private final boolean isAuto;

    public TrackGoal(boolean isAuto) {
        this.isAuto = isAuto;
        this.drive = DriveTrain.getInstance();
        this.lime = Limelight.getInstance();
        this.gyro = Gyro.getInstance();
        if (isAuto) {
            addRequirements(drive);
        }
    }

    public double getTargetAngle() {
        return gyro.getRotation2d().getDegrees() - lime.getAdjustedTxAndCalc();
    }

    @Override
    public void initialize() {
        drive.setUseDriverAssist(true);
    }

    @Override
    public void execute() {
        if (lime.isDetected()) {
            if (isAuto) {
                drive.drive(0, 0, drive.getRotationSpeed(), true);
            }

            var newAngle = getTargetAngle();

            if (MathUtil.isWithinTolerance(newAngle, drive.getTargetRotationAngle(), TOLERANCE)) {
                drive.setTargetRotationAngle(newAngle);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setUseDriverAssist(false);
        if (isAuto) {
            drive.drive(0, 0, 0, true);
        }
    }
}
