package org.rivierarobotics.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        return limelightTable.getEntry("targetPitch").getDouble(0);
    }

    public double getTx() {
        return limelightTable.getEntry("targetYaw").getDouble(0);
    }

    public boolean getDetected(){
        return limelightTable.getEntry("hasTarget").getBoolean(false);
    }

    public double getDistance(){
        return Math.atan(Math.toRadians(getTy()))*(ShooterConstants.getGoalHeight() - ShooterConstants.getRobotHeight());
    }

    @Override
    public void periodic() {
    }


}
