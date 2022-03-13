package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.swerve.TrajectoryFollower;

public class DrivePath extends CommandBase {
    private final String path;
    private final DriveTrain driveTrain;
    private TrajectoryFollower trajectoryFollower;

    public DrivePath (String path) {
        this.path = path;
        this.driveTrain = DriveTrain.getInstance();
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        this.trajectoryFollower = new TrajectoryFollower(TrajectoryFollower.getTrajectoryFromPathweaver(path), true, Gyro.getInstance(), DriveTrain.getInstance());
    }

    @Override
    public void execute() {
        this.trajectoryFollower.followController();
    }

    @Override
    public boolean isFinished() {
        return trajectoryFollower.isFinished();
    }
}

