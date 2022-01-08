package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.swerveDrive.DriveTrain;

public class SetDriveVoltage extends InstantCommand {
    private final DriveTrain dt;
    private final double v;
    public SetDriveVoltage(double v) {
        this.dt = DriveTrain.getInstance();
        this.v = v;
    }

    @Override
    public void initialize() {
        dt.testSetVoltage(v);
    }
}