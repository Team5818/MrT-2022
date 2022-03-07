package org.rivierarobotics.commands.limelight;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.robot.ControlMap;
import org.rivierarobotics.subsystems.vision.Hood;
import pabeles.concurrency.IntOperatorTask;

public class ShooterControl extends CommandBase {
    private final Hood hood;
    private final Joystick joystick;
    private final double MaxVoltage = 10;

    public ShooterControl(){
        this.hood = Hood.getInstance();
        this.joystick = ControlMap.DRIVER_RIGHT;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        var voltage = MathUtil.fitDeadband(joystick.getY()) * MaxVoltage;
        hood.setActuatorVoltage(voltage);
    }
}
