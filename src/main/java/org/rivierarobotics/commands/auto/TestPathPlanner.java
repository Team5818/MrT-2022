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

package org.rivierarobotics.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.swerve.TrajectoryFollower;

//TODO the only difference b/n this and DrivePath is PathPlanner.loadPath() instead of TrajectoryFollower.getTrajectoryFromPathweaver()
// please combine, but put a boolean flag in the constructor for DrivePath (i.e. usePathPlanner)
// (and then delete this if no longer needed, ofc moving usages
public class TestPathPlanner extends CommandBase {
    private final DriveTrain driveTrain;
    private final Gyro gyro;
    private final String trajectoryJSON;
    private TrajectoryFollower trajectoryFollower;

    public TestPathPlanner(String path) {
        this.trajectoryJSON = path;
        this.driveTrain = DriveTrain.getInstance();
        this.gyro = Gyro.getInstance();
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        this.trajectoryFollower = new TrajectoryFollower(
                PathPlanner.loadPath(trajectoryJSON, 2, 0.75, false),
                true, gyro, driveTrain
        );
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
