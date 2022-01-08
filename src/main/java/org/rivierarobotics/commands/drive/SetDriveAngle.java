package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.swerveDrive.DriveTrain;
import org.rivierarobotics.util.Gyro;


public class SetDriveAngle extends CommandBase {
    private final DriveTrain dt;
    private final Gyro gyro;
    private final double angle;

    public SetDriveAngle(double angle) {
        this.dt = DriveTrain.getInstance();
        this.gyro = Gyro.getInstance();
        this.angle = angle;
        addRequirements(this.dt);
    }

    @Override
    public void execute() {
        if (angle >= 0) dt.drive(0, 0, DriveTrain.MAX_ANGULAR_SPEED, false);
        else dt.drive(0, 0, -DriveTrain.MAX_ANGULAR_SPEED, false);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(gyro.getAngle(),angle,1);
    }
}
