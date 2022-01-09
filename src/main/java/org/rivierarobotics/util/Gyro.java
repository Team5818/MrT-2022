package org.rivierarobotics.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
    private final AHRS navX;
    private static Gyro gyro;

    public static Gyro getInstance() {
        if (gyro == null) {
            gyro = new Gyro();
        }
        return gyro;
    }

    public Gyro() {
        this.navX = new AHRS(SPI.Port.kMXP);
    }

    public double getAngle() {
        return navX.getAngle();
    }

    public Rotation2d getRotation2d() {
        SmartDashboard.putNumber("GG", getAngle());
        SmartDashboard.putNumber("GA", new Rotation2d(Math.toRadians(getAngle())).getDegrees());
        return new Rotation2d(Math.toRadians(getAngle()));
    }

    public double getRate() {
        return Math.toRadians(navX.getRate());
    }

    public void setAngleAdjustment(double angle) {
        navX.reset();
        navX.setAngleAdjustment(angle);
    }

    public void resetGyro() {
        navX.reset();
    }
}