package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.aifield.AIFieldDisplay;
import org.rivierarobotics.util.aifield.FieldMesh;
import org.rivierarobotics.util.aifield.FieldNode;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class DriveToClosest extends SequentialCommandGroup {
    private final BoundingBox defaultBallBox = new BoundingBox(0,0,0,0);
    private final DriveTrain driveTrain;
    private final FieldMesh aiFieldMesh;


    public DriveToClosest() {
        this.driveTrain = DriveTrain.getInstance();
        this.aiFieldMesh = FieldMesh.getInstance();
    }

    @Override
    public void initialize() {
        var currentX = driveTrain.getRobotPose().getX();
        var currentY = driveTrain.getRobotPose().getY();


        MLCore core = MLCore.getInstance();
        MLObject ball = new MLObject("red", defaultBallBox, 1);

        try {
            ball = core.getDetectedObjects().get("red").get(0);
        } catch (NullPointerException nullPointerException){
            return;
        }

        Logging.robotShuffleboard.getTab("ML").setEntry("Target BallX", ball.relativeLocationY);
        Logging.robotShuffleboard.getTab("ML").setEntry("Target BallY", ball.relativeLocationX);
        Logging.robotShuffleboard.getTab("ML").setEntry("Target Ball Distance", ball.relativeLocationDistance);
        Logging.robotShuffleboard.getTab("ML").setEntry("TX", ball.ty);
        Logging.robotShuffleboard.getTab("ML").setEntry("TY", ball.tx);

        var targetX = currentX - Math.cos(Gyro.getInstance().getAngle() + ball.tx) * ball.relativeLocationDistance;
        var targety = currentY + Math.sin(Gyro.getInstance().getAngle() + ball.tx) * ball.relativeLocationDistance;

        Logging.robotShuffleboard.getTab("ML").setEntry("CurrentX", currentX);
        Logging.robotShuffleboard.getTab("ML").setEntry("CurrentY", currentY);
        Logging.robotShuffleboard.getTab("ML").setEntry("targetX", targetX);
        Logging.robotShuffleboard.getTab("ML").setEntry("targetY", targety);

        var trajectory = aiFieldMesh.getTrajectory(currentX, currentY, targetX, targety, true, 0, driveTrain.getSwerveDriveKinematics());

//        Logging.aiFieldDisplay.updatePath(trajectory);
//        if (trajectory != null) {
//            driveTrain.drivePath(trajectory);
//        }
    }

//    @Override
//    public boolean isFinished() {
//        return !driveTrain.followHolonomicController();
//    }
}
