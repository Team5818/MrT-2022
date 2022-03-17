/*
 * This file is part of Placeholder-2022, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.util.smartmotion;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.lib.MotionMagicConfig;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.lib.shuffleboard.RSTab;
import org.rivierarobotics.robot.Logging;


public class SparkSmartMotion {
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;
    private final RSTab tuningTab;
    private final SparkMaxPIDController pidController;

    /**
     *
     * Constructs a SparkSmartMotion object for live tuning. Make sure to call
     * checkForPIDChanges periodically for this to work.
     *
     * @param motor SparkMax Motor
     * @param pidConfig PID Config
     * @param sparkMotionConfig SparkMax Motion Config
     * @param tuningTab Tab on the shuffleboard to put tuning values on
     */
    public SparkSmartMotion(CANSparkMax motor, PIDConfig pidConfig, SparkMotionConfig sparkMotionConfig, RSTab tuningTab) {
        this.pidController = setupSmartMotion(motor, pidConfig, sparkMotionConfig);

        kP = pidConfig.getP();
        kI = pidConfig.getI();
        kD = pidConfig.getD();
        kIz = sparkMotionConfig.getIntegralZone();
        kFF = pidConfig.getF();
        kMaxOutput = pidConfig.getRange();
        kMinOutput = -pidConfig.getRange();
        minVel = sparkMotionConfig.getMinVel();
        maxVel = sparkMotionConfig.getMaxVel();
        maxAcc = sparkMotionConfig.getMaxAccel();
        allowedErr = pidConfig.getTolerance();

        this.tuningTab = tuningTab;


        // display PID coefficients on Shuffleboard tab
        tuningTab.setEntry("P Gain", kP);
        tuningTab.setEntry("I Gain", kI);
        tuningTab.setEntry("D Gain", kD);
        tuningTab.setEntry("I Zone", kIz);
        tuningTab.setEntry("Feed Forward", kFF);
        tuningTab.setEntry("Max Output", kMaxOutput);
        tuningTab.setEntry("Min Output", kMinOutput);

        // display Smart Motion coefficients
        tuningTab.setEntry("Max Velocity", maxVel);
        tuningTab.setEntry("Min Velocity", minVel);
        tuningTab.setEntry("Max Acceleration", maxAcc);
        tuningTab.setEntry("Allowed Closed Loop Error", allowedErr);
        tuningTab.setEntry("Set Position", 0);
        tuningTab.setEntry("Set Velocity", 0);
    }

    public void checkForPIDChanges() {
        double p = tuningTab.getEntry("P Gain").getDouble(0);
        double i = tuningTab.getEntry("I Gain").getDouble(0);
        double d = tuningTab.getEntry("D Gain").getDouble(0);
        double iz = tuningTab.getEntry("I Zone").getDouble(0);
        double ff = tuningTab.getEntry("Feed Forward").getDouble(0);
        double max = tuningTab.getEntry("Max Output").getDouble(0);
        double min = tuningTab.getEntry("Min Output").getDouble(0);
        double maxV = tuningTab.getEntry("Max Velocity").getDouble(0);
        double minV = tuningTab.getEntry("Min Velocity").getDouble(0);
        double maxA = tuningTab.getEntry("Max Acceleration").getDouble(0);
        double allE = tuningTab.getEntry("Allowed Closed Loop Error").getDouble(0);

        if (p != kP) {
            pidController.setP(p);
            kP = p;
        }
        if (i != kI) {
            pidController.setI(i);
            kI = i;
        }
        if (d != kD) {
            pidController.setD(d);
            kD = d;
        }
        if (iz != kIz) {
            pidController.setIZone(iz);
            this.kIz = iz;
        }
        if (ff != kFF) {
            pidController.setFF(ff);
            this.kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            pidController.setOutputRange(min, max);
            this.kMinOutput = min;
            this.kMaxOutput = max;
        }
        if (maxV != maxVel) {
            pidController.setSmartMotionMaxVelocity(maxV, 0);
            this.maxVel = maxV;
        }
        if (minV != minVel) {
            pidController.setSmartMotionMinOutputVelocity(minV, 0);
            this.minVel = minV;
        }
        if (maxA != maxAcc) {
            pidController.setSmartMotionMaxAccel(maxA, 0);
            this.maxAcc = maxA;
        }
        if (allE != allowedErr) {
            pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
            this.allowedErr = allE;
        }
    }

    public SparkMaxPIDController getPidController(){
        return pidController;
    }

    /**
     * Configures a SparkMax motor to use Smart Motion.
     *
     * @param motor motor to configure
     * @param pidConfig pid config
     * @param sparkMotionConfig spark max specific config
     * @return pid controller
     */
    public static SparkMaxPIDController setupSmartMotion(CANSparkMax motor, PIDConfig pidConfig, SparkMotionConfig sparkMotionConfig) {
        motor.restoreFactoryDefaults();
        var mPidController = motor.getPIDController();

        mPidController.setP(pidConfig.getP());
        mPidController.setI(pidConfig.getI());
        mPidController.setD(pidConfig.getD());
        mPidController.setIZone(sparkMotionConfig.getIntegralZone());
        mPidController.setFF(pidConfig.getF());
        mPidController.setOutputRange(-pidConfig.getRange(), pidConfig.getRange());

        mPidController.setSmartMotionMaxVelocity(sparkMotionConfig.getMaxVel(), 0);
        mPidController.setSmartMotionMinOutputVelocity(sparkMotionConfig.getMinVel(), 0);
        mPidController.setSmartMotionMaxAccel(sparkMotionConfig.getMaxAccel(), 0);
        mPidController.setSmartMotionAllowedClosedLoopError(pidConfig.getTolerance(), 0);

        motor.burnFlash();
        return mPidController;
    }
}
