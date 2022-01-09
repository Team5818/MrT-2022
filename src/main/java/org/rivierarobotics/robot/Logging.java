package org.rivierarobotics.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.rivierarobotics.lib.shuffleboard.RobotShuffleboard;

public class Logging {
    public static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    public static RobotShuffleboard robotShuffleboard = new RobotShuffleboard();

    private Logging() {

    }

    public static void initialize() {
        networkTableInstance = NetworkTableInstance.getDefault();
        robotShuffleboard = new RobotShuffleboard();
    }
}