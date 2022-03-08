package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.robot.ControlMap;
import org.rivierarobotics.subsystems.intake.Intake;

public class CollectControl extends CommandBase {
    private final Intake intake;
    private final Joystick rightJoystick;

    public CollectControl(){
        this.intake = Intake.getInstance();
        this.rightJoystick = ControlMap.CO_DRIVER_RIGHT;
        intake.setIntakeState(false);
        addRequirements(intake);
    }

    @Override
    public void execute() {
        var voltage = MathUtil.fitDeadband(rightJoystick.getY()) * 10;
        //intake.setIntakeState(false);
        SmartDashboard.putNumber("beltvoltage", voltage);
        intake.setVoltages(voltage, voltage);
    }
}
