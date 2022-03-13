package org.rivierarobotics.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class Sideways extends CommandBase {
    private final double time;
    private double starttime = 0;
    public Sideways(double time) {
        this.time = time;
        addRequirements(DriveTrain.getInstance());
    }

    @Override
    public void initialize() {
        starttime = Timer.getFPGATimestamp();
        DriveTrain.getInstance().drive(0,1,0, true);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - starttime > time;
    }

    @Override
    public void end(boolean interrupted) {
        DriveTrain.getInstance().drive(0,0,0,true);
    }
}
