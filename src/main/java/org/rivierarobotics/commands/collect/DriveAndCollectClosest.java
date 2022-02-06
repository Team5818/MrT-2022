package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class DriveAndCollectClosest extends CommandBase {
    public MLObject ball;
    public final MLCore mlCore;
    public boolean doesBallExist;
    public final BoundingBox defaultBallBox = new BoundingBox(0,0,0,0);

    public DriveAndCollectClosest() {
        this.mlCore = MLCore.getInstance();
        this.doesBallExist = true;
        this.ball = new MLObject("red", defaultBallBox,1);

        try {
            this.ball = mlCore.getDetectedObjects().get("red").get(0);
        } catch (NullPointerException nullPointerException){
            return;
        }

    }

    @Override
    public void execute() {
//        new SequentialCommandGroup(new DriveToPoint(ball.fieldLocationX, ball.fieldLocationY, true, 0));
    }
}
