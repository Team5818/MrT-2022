package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class CollectVisionTest extends CommandBase {
//    public MLObject ball;
//    public final MLCore mlCore;
    public final BoundingBox defaultBallBox = new BoundingBox(0,0,0,0);

    public CollectVisionTest() {
//        this.mlCore = MLCore.getInstance();
//        this.ball = new MLObject("red", defaultBallBox,1);
//
//        try {
//            this.ball = mlCore.getDetectedObjects().get("red").get(0);
//        } catch (NullPointerException nullPointerException){
//            return;
//        }

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

        Logging.robotShuffleboard.getTab("ML").setEntry("Target BallX", ball.relativeLocationX);
        Logging.robotShuffleboard.getTab("ML").setEntry("Target BallY", ball.relativeLocationY);
        Logging.robotShuffleboard.getTab("ML").setEntry("Target Ball Distance", ball.relativeLocationDistance);
        Logging.robotShuffleboard.getTab("ML").setEntry("TX", ball.tx);
        Logging.robotShuffleboard.getTab("ML").setEntry("TY", ball.ty);
    }
}
