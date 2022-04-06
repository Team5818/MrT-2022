package org.rivierarobotics.commands.advanced.shoot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.ml.MLCore;

import java.util.Comparator;

public class RotateBall extends CommandBase {
    private final DriveTrain driveTrain;
    public RotateBall() {
        this.driveTrain = DriveTrain.getInstance();
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        MLCore core = MLCore.getInstance();
        var ballColor = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? "blue" : "red";

        var balls = core.getDetectedObjects().get(ballColor);
        if (balls == null || balls.isEmpty()) {
            return;
        }

        balls.sort(Comparator.comparingDouble(a -> a.relativeLocationDistance));

        driveTrain.setTargetRotationAngle(Gyro.getInstance().getRotation2d().getDegrees() + balls.get(0).ty);
        driveTrain.drive(0,0,driveTrain.getRotationSpeed(), true);
    }

    @Override
    public boolean isFinished() {
        return driveTrain.getRotationSpeed() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(0,0,0,true);
    }
}
