package org.rivierarobotics.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTable limelightTable;

    private static Limelight limelight;


    public static Limelight getInstance() {
        if (limelight == null) {
            limelight = new Limelight();
        }
        return limelight;
    }

    public Limelight(){
        this.limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getTy() {
        return limelightTable.getEntry("ty").getDouble(-1);
    }

    public double getTx() {
        return limelightTable.getEntry("tx").getDouble(-1);
    }

    public double getArea(){
        return limelightTable.getEntry("ta").getDouble(-1);
    }

    public boolean getDetected(){
        return limelightTable.getEntry("tv").getDouble(-1) == 0;
    }

    public double getDistance(){
        return Math.atan(Math.toRadians(getTy()))*(ShooterConstants.getGoalHeight() - ShooterConstants.getRobotHeight());
    }


}
