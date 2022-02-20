package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.aifield.AIFieldDisplay;
import org.rivierarobotics.util.aifield.FieldMesh;
import org.rivierarobotics.util.aifield.FieldNode;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class DriveAndCollectClosest extends CommandBase {
    private final BoundingBox defaultBallBox = new BoundingBox(0,0,0,0);
    private final DriveTrain driveTrain;
    private final FieldMesh aiFieldMesh;


    public DriveAndCollectClosest() {
        this.driveTrain = DriveTrain.getInstance();
        this.aiFieldMesh = FieldMesh.getInstance();
    }

    @Override
    public void initialize() {
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

        var dtPose = driveTrain.getRobotPose();
        //if(MathUtil.isWithinTolerance(dtPose.getX(),0,0.3) && MathUtil.isWithinTolerance(dtPose.getY(),0,0.3)) return;
        var trajectory = aiFieldMesh.getTrajectory(0, 0, 1, 1, true, 0, DriveTrain.getInstance().getSwerveDriveKinematics());
        Logging.aiFieldDisplay.updatePath(trajectory);
        if (trajectory != null) {
            driveTrain.drivePath(trajectory);
        }

    }

    @Override
    public boolean isFinished() {
        return !driveTrain.followHolonomicController();
    }
}
