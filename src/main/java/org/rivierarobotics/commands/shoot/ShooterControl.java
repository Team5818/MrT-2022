package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.robot.ControlMap;
import org.rivierarobotics.subsystems.shoot.Floppas;

public class ShooterControl extends CommandBase {
    private final Floppas floppas;
    private final Joystick joystick;
    private final double MaxVoltage = 10;

    public ShooterControl(){
        this.floppas = Floppas.getInstance();
        this.joystick = ControlMap.CO_DRIVER_RIGHT;
        addRequirements(floppas);
    }

    @Override
    public void execute() {
        var voltage = MathUtil.fitDeadband(joystick.getY()) * MaxVoltage;
        floppas.setActuatorVoltage(voltage);
    }
}
