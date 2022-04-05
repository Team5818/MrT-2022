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

package org.rivierarobotics.commands.advanced.drive;

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
        var trajectory = MLCore.getBallTrajectory(driveTrain, gyro, aiFieldMesh);
        this.trajectoryFollower = new TrajectoryFollower(trajectory, false, gyro, driveTrain);
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
