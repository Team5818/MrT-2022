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
import org.rivierarobotics.lib.shuffleboard.RSTab;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.aifield.FieldMesh;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.swerve.TrajectoryFollower;

import java.util.Comparator;

public class DriveToClosest extends SequentialCommandGroup {
    private static final double X_SPEED = 1;
    private static final double Y_SPEED = 1;
    private final DriveTrain driveTrain;
    private final Gyro gyro;
    private final FieldMesh aiFieldMesh;
    private final RSTab shuffleboard;
    private TrajectoryFollower trajectoryFollower;

    public DriveToClosest() {
        this.driveTrain = DriveTrain.getInstance();
        this.aiFieldMesh = FieldMesh.getInstance();
        this.gyro = Gyro.getInstance();
        this.shuffleboard = Logging.robotShuffleboard.getTab("drive");
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        var trajectory = MLCore.trajectory != null ? MLCore.trajectory : MLCore.getBallTrajectory(driveTrain, gyro, aiFieldMesh);
        this.trajectoryFollower = new TrajectoryFollower(trajectory, false, gyro, driveTrain);
    }

    @Override
    public void execute() {
        MLCore core = MLCore.getInstance();
        var ballColor = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? "blue" : "red";

        var balls = core.getDetectedObjects().get(ballColor);
        if (balls == null || balls.isEmpty()) {
            return;
        }

        balls.sort(Comparator.comparingDouble(a -> a.relativeLocationDistance));
        var ball = balls.get(0);
        shuffleboard.setEntry("tx", ball.tx).setEntry("ty", ball.ty);
        driveTrain.setTargetRotationAngle(Gyro.getInstance().getRotation2d().getDegrees() + ball.ty);
        driveTrain.drive(X_SPEED, ball.tx >= 0 ? Y_SPEED : -Y_SPEED, driveTrain.getRotationSpeed(), false);
        driveTrain.drive(0, 0, driveTrain.getRotationSpeed(), true);
        trajectoryFollower.followController();
    }

    @Override
    public boolean isFinished() {
        return trajectoryFollower.isFinished();
    }
}
