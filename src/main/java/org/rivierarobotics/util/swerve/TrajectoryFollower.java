package org.rivierarobotics.util.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;

import java.io.IOException;
import java.nio.file.Path;

public class TrajectoryFollower {
    private final HolonomicDriveController holonomicDriveController;
    private final double startTime;
    private final PoseEstimator estimator;
    private final Trajectory trajectory;
    private final Gyro gyro;
    private final DriveTrain driveTrain;

    public static Trajectory getTrajectoryFromPathweaver(String path) {
        String trajectoryJSON = "paths/" + path + ".wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        try {
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Pathweaver path not found", e.getStackTrace());
        }
        return null;
    }

    public TrajectoryFollower(Trajectory trajectory, boolean resetPose, Gyro gyro, DriveTrain driveTrain) {
        this.holonomicDriveController = driveTrain.getHolonomicDriveController();
        this.startTime = Timer.getFPGATimestamp();
        this.gyro = gyro;
        this.driveTrain = driveTrain;
        this.estimator = driveTrain.getPoseEstimator();
        this.trajectory = trajectory;
        if(trajectory == null) {
            DriverStation.reportError("Trying to drive null path", false);
            return;
        }
        if(resetPose) {
            var transposedPose = new Pose2d(trajectory.getInitialPose().getTranslation(), gyro.getRotation2d());
            estimator.resetPose(transposedPose);
        }
    }

    public boolean isFinished() {
        return trajectory == null || (Timer.getFPGATimestamp() - this.startTime > this.trajectory.getTotalTimeSeconds());
    }

    /**
     * Call this method periodically to follow a trajectory.
     * returns false when path is done.
     */
    public void followController() {
        if(trajectory == null) {
            return;
        }

        double timePassed = Timer.getFPGATimestamp() - this.startTime;

        var state = this.trajectory.sample(timePassed);
        var controls = holonomicDriveController.calculate(
                new Pose2d(estimator.getRobotPose().getTranslation(), new Rotation2d(0)),
                state,
                new Rotation2d(0)
        );

        driveTrain.drive(controls.vxMetersPerSecond, controls.vyMetersPerSecond, 0, true);
    }
}
