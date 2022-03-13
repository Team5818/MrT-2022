package org.rivierarobotics.commands.drive;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.aifield.FieldMesh;

public class DriveToPoint extends CommandBase {
    private final DriveTrain driveTrain;
    private final Gyro gyro;
    private final FieldMesh aiFieldMesh;
    private final double targetY;
    private final double targetX;
    private final boolean shouldStop;
    private final double initialVelocity;

    public DriveToPoint(double targetX, double targetY, boolean shouldStop, double initialVelocity) {
        this.driveTrain = DriveTrain.getInstance();
        this.aiFieldMesh = FieldMesh.getInstance();
        this.gyro = Gyro.getInstance();
        this.targetY = targetY;
        this.targetX = targetX;
        this.shouldStop = shouldStop;
        this.initialVelocity = initialVelocity;
        addRequirements(driveTrain);
    }

    //TODO: Surround all of this in a try/catch just in case a trajectory is invalid.
    @Override
    public void initialize() {
        var dtPose = driveTrain.getRobotPose();
        var trajectory = aiFieldMesh.getTrajectory(dtPose.getX(), dtPose.getY(), targetX, targetY, shouldStop, initialVelocity, DriveTrain.getInstance().getSwerveDriveKinematics());
        Logging.aiFieldDisplay.updatePath(trajectory);
        if(trajectory != null) driveTrain.drivePath(trajectory);
    }

    @Override
    public boolean isFinished() {
        return !driveTrain.followHolonomicController();
    }
}
