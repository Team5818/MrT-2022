package org.rivierarobotics.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.swerve.TrajectoryFollower;

public class TestPathPlanner extends CommandBase {
    private final DriveTrain driveTrain;
    private final Gyro gyro;
    private final String trajectoryJSON;
    private TrajectoryFollower trajectoryFollower;

    public TestPathPlanner(String path){
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
