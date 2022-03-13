package org.rivierarobotics.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.ClimbPistons;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class SubsystemMap {
    public DriveTrain driveTrain;

    public Climb climb;
    public ClimbPistons climbPistons;



    public SubsystemMap() {

    }


}
