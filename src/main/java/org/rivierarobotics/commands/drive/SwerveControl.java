package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.robot.ControlMap;
import org.rivierarobotics.subsystems.swerveDrive.DriveTrain;

public class SwerveControl extends CommandBase {
    private final DriveTrain drivetrain;
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;
    public SwerveControl() {
        this.drivetrain = DriveTrain.getInstance();
        this.leftJoystick = ControlMap.driverLeft;
        this.rightJoystick = ControlMap.driverRight;
        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        var xSpeed = MathUtil.fitDeadband(-leftJoystick.getY()) * DriveTrain.MAX_SPEED;
        var ySpeed = MathUtil.fitDeadband(leftJoystick.getX()) * DriveTrain.MAX_SPEED;
        var rot = MathUtil.fitDeadband(rightJoystick.getX()) * DriveTrain.MAX_ANGULAR_SPEED;

        drivetrain.drive(xSpeed, ySpeed, rot, true);
    }
}
