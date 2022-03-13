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

package org.rivierarobotics.subsystems.swervedrive;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import org.rivierarobotics.lib.MotionMagicConfig;
import org.rivierarobotics.lib.MotorUtil;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.util.StatusFrameDemolisher;
import org.rivierarobotics.util.swerve.SwerveUtil;

import java.util.ArrayList;

public class SwerveModule {
    //Physical Constants
    private static final double WHEEL_RADIUS = 0.03915;
    private static final int ENCODER_RESOLUTION = 4096;
    private static final double STEER_MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION; // radians
    private static final double GEARING = 11.0 / 40.0;
    private static final double DRIVE_MOTOR_TICK_TO_SPEED = 10 * GEARING * (2 * Math.PI * WHEEL_RADIUS) / 2048; // m/s

    private final double zeroTicks;

    private static final double MAX_TURN_ACCELERATION = 30000; //Rad/s
    private static final double MAX_TURN_VELOCITY = 30000; //Rad/s
    private static final int TIMEOUT_MS = 60;


    //Turn Motor Motion Magic
    private static final MotionMagicConfig TM_MM_CONFIG = new MotionMagicConfig(
            new ArrayList<>(), true,
            MAX_TURN_VELOCITY, MAX_TURN_ACCELERATION,
            100, 2,
            TIMEOUT_MS, 10
    );
    private static final PIDConfig TM_MM_PID = new PIDConfig(0.7, 0, 0, 0.1);
    //Drive Motor Motion Magic
    private static final MotionMagicConfig DM_MM_CONFIG = new MotionMagicConfig(
            new ArrayList<>(), true,
            DriveTrain.MAX_SPEED, DriveTrain.MAX_ACCELERATION,
            100, 2,
            TIMEOUT_MS, 10
    );
    private static final PIDConfig DM_MM_PID = new PIDConfig(0.0, 0, 0, 0.1);


    private final WPI_TalonFX driveMotor;
    private final WPI_TalonSRX steeringMotor;



    private double prevAngle = 0.0;
    private double timeoutSeconds = 0.0;
    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel    ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     * @param zeroTicks            ticks when angle = 0
     */
    public SwerveModule(int driveMotorChannel, int steeringMotorChannel, double zeroTicks) {
        this.steeringMotor = new WPI_TalonSRX(steeringMotorChannel);
        this.zeroTicks = zeroTicks;

        //Steer Motor
        steeringMotor.configFactoryDefault(TIMEOUT_MS);
        steeringMotor.configFeedbackNotContinuous(true, TIMEOUT_MS);
        steeringMotor.setSensorPhase(false);
        steeringMotor.setInverted(true);
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition, TM_MM_PID, DM_MM_CONFIG, steeringMotor);
        StatusFrameDemolisher.demolishStatusFrames(steeringMotor, false);

        //Drive Motor
        this.driveMotor = new WPI_TalonFX(driveMotorChannel);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 20);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition, DM_MM_PID, DM_MM_CONFIG, steeringMotor);
        StatusFrameDemolisher.demolishStatusFrames(driveMotor, false);

        //Current Limits
        this.driveMotor.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0.05));
        this.steeringMotor.configContinuousCurrentLimit(30);
        this.steeringMotor.configPeakCurrentLimit(30);
    }

    public double getAbsoluteAngle() {
        return SwerveUtil.clampAngle(getAngle());
    }

    public double getAngle() {
        timeoutSeconds += Timer.getFPGATimestamp() - timeoutSeconds;
        var currAngle = (steeringMotor.getSelectedSensorPosition() - zeroTicks) * STEER_MOTOR_TICK_TO_ANGLE;
        if (prevAngle == 0) prevAngle = currAngle;
        if (timeoutSeconds <= 1 && Math.abs(prevAngle - currAngle) >= 12 * Math.PI) {
            return prevAngle;
        }
        if (timeoutSeconds > 1) {
            timeoutSeconds = 0;
        }
        prevAngle = currAngle;
        return currAngle;
    }

    public double convertAngleToTick(double angleInRads) {
        return (angleInRads / STEER_MOTOR_TICK_TO_ANGLE) + zeroTicks;
    }

    public double convertVelocityToTicksPer100ms(double velocity) {
        return velocity / DRIVE_MOTOR_TICK_TO_SPEED;
    }

    public double getPosTicks() {
        return steeringMotor.getSelectedSensorPosition();
    }

    public double getDriveTicks() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return driveMotor.getSensorCollection().getIntegratedSensorVelocity() * DRIVE_MOTOR_TICK_TO_SPEED;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(SwerveUtil.clampAngle(getAngle())));
    }

    public void setDriveMotorVelocity(double metersPerSecond) {
        driveMotor.set(TalonFXControlMode.Velocity, convertVelocityToTicksPer100ms(metersPerSecond));
    }

    public void setSteeringMotorAngle(double angleInRad) {
        steeringMotor.set(ControlMode.MotionMagic, angleInRad);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        var optimizedAngle = SwerveUtil.optimizeSwerveStates(state, getAngle());
        setDriveMotorVelocity(optimizedAngle.speedMetersPerSecond);
        setSteeringMotorAngle(convertAngleToTick(optimizedAngle.angle.getRadians()));
    }
}
