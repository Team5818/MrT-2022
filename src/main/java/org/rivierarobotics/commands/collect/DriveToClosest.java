package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj.DriverStation;
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

import java.util.Comparator;

public class DriveToClosest extends SequentialCommandGroup {
    private final BoundingBox defaultBallBox = new BoundingBox(0,0,0,0);
    private final DriveTrain driveTrain;
    private final Gyro gyro;
    private final FieldMesh aiFieldMesh;


    public DriveToClosest() {
        this.driveTrain = DriveTrain.getInstance();
        this.aiFieldMesh = FieldMesh.getInstance();
        this.gyro = Gyro.getInstance();
    }

    @Override
    public void initialize() {
        gyro.resetGyro();

        var currentX = driveTrain.getRobotPose().getX();
        var currentY = driveTrain.getRobotPose().getY();


        MLCore core = MLCore.getInstance();
        var ballColor = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? "blue" : "red";
        var redBalls = MLCore.getInstance().getDetectedObjects().get("red");
        if(redBalls == null || redBalls.isEmpty()) {
            return;
        }

        redBalls.sort(Comparator.comparingDouble(a -> a.relativeLocationDistance));
        var ball = redBalls.get(0);

        Logging.robotShuffleboard.getTab("ML").setEntry("Target BallY", ball.relativeLocationY);
        Logging.robotShuffleboard.getTab("ML").setEntry("Target BallX", ball.relativeLocationX);
        Logging.robotShuffleboard.getTab("ML").setEntry("Target Ball Distance", ball.relativeLocationDistance);
        Logging.robotShuffleboard.getTab("ML").setEntry("Target ty", ball.ty);
        Logging.robotShuffleboard.getTab("ML").setEntry("Target tx", ball.tx);

        var targetX = currentX + Math.cos(Gyro.getInstance().getRotation2d().getRadians() + Math.toRadians(ball.ty)) * ball.relativeLocationDistance;
        var targety = currentY + Math.sin(Gyro.getInstance().getRotation2d().getRadians() + Math.toRadians(ball.ty)) * ball.relativeLocationDistance;

        Logging.robotShuffleboard.getTab("ML").setEntry("CurrentX", currentX);
        Logging.robotShuffleboard.getTab("ML").setEntry("CurrentY", currentY);
        Logging.robotShuffleboard.getTab("ML").setEntry("targetX", targetX);
        Logging.robotShuffleboard.getTab("ML").setEntry("targetY", targety);

        var trajectory = aiFieldMesh.getTrajectory(currentX, currentY, targetX,  targety, true, 0, driveTrain.getSwerveDriveKinematics());

        Logging.aiFieldDisplay.updatePath(trajectory);
//        if (trajectory != null) {
//            driveTrain.drivePath(trajectory);
//        }
    }

    @Override
    public boolean isFinished() {
        return true;
        //return !driveTrain.followHolonomicController();
    }
}
