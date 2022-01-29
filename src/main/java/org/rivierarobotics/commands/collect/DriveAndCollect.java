package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.auto.DriveToPoint;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.ml.MLObject;

public class DriveAndCollect extends CommandBase {
    public final MLObject ball;

    public DriveAndCollect(MLObject ball) {
        this.ball = ball;
    }

    @Override
    public void execute() {
        new SequentialCommandGroup(new DriveToPoint(ball.fieldLocationX, ball.fieldLocationY, true, 0));
    }
}
