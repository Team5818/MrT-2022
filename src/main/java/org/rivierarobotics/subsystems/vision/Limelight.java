package org.rivierarobotics.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
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
    //REPLACE WITH HOOD ANGLE
    double hoodTest = 0.5;
    public double getDistance(){

        double hoodAngle = hoodTest;
        double llHeight = Math.sin(Math.toRadians(hoodAngle)) * ShooterConstants.getLLtoTurretY() + ShooterConstants.getRobotHeight();
        double llTy = getTy() + hoodAngle;
        double llDist = (ShooterConstants.getGoalHeight() - llHeight) / Math.tan(Math.toRadians(llTy));
        double dist = llDist + Math.cos(Math.toRadians(hoodAngle)) * ShooterConstants.getLLtoTurretY();

        return Math.toDegrees(Math.atan((ShooterConstants.getGoalHeight() - llHeight) / dist));
    }

}
