package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.commands.shoot.TrackGoal;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class TrackBall extends CommandBase {
    private final DriveTrain driveTrain;
    private final MLCore mlCore;

    public TrackBall(){
        this.driveTrain = DriveTrain.getInstance();
        this.mlCore = MLCore.getInstance();
    }

    @Override
    public void execute() {
        MLObject ball = new MLObject(MLCore.TARGET_COLOR, new BoundingBox(0,0,0,0), 10);
        try {
            ball = mlCore.getDetectedObjects().get(MLCore.TARGET_COLOR).get(0);
            driveTrain.setTargetRotationAngle(ball.tx + driveTrain.getRobotPose().getRotation().getRadians());
        } catch (Exception e) {
            return;
        }

    }
}
