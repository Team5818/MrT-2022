/*
 * This file is part of MrT-2022, licensed under the GNU General Public License (GPLv3).
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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.rivierarobotics.lib.MotionMagicConfig;
import org.rivierarobotics.lib.MotorUtil;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.StatusFrameDemolisher;
import org.rivierarobotics.util.swerve.SwerveUtil;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class SwerveModule {
    //Physical Constants
    private static final double WHEEL_RADIUS = 0.03915;
    private static final int ENCODER_RESOLUTION = 4096;
    private static final double STEER_MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION; // radians
    private static final double GEARING = 11.0 / 40.0;
    private static final double DRIVE_MOTOR_TICK_TO_SPEED = 10 * GEARING * (2 * Math.PI * WHEEL_RADIUS) / 2048; // m/s
    //Controller Constants
    private static final double MAX_TURN_ACCELERATION = 4800; //Rad/s
    private static final double MAX_TURN_VELOCITY = 4800; //Rad/s
    private static final int TIMEOUT_MS = 60;

    //Turn Motor Motion Magic
    private static final MotionMagicConfig TM_MM_CONFIG = new MotionMagicConfig(
            new ArrayList<>(), true,
            MAX_TURN_VELOCITY, MAX_TURN_ACCELERATION,
            150, 0,
            TIMEOUT_MS, 10
    );
    private static final PIDConfig TM_MM_PID = new PIDConfig(3.5, 0.001, 0, 0);

    //Drive Motor Motion Magic
    private static final MotionMagicConfig DM_MM_CONFIG = new MotionMagicConfig(
            new ArrayList<>(), true,
            DriveTrain.MAX_TURN_SPEED, DriveTrain.MAX_ACCELERATION,
            300, 2,
            TIMEOUT_MS, 10
    );
    private static final PIDConfig DM_MM_PID = new PIDConfig(0.0026, 0.0001, 0, 0.06);

    private final double zeroTicks;

    //Motors
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonSRX steeringMotor;

    //Thread-Safe angles to reduce CAN usage
    private final AtomicReference<Double> swerveAngle = new AtomicReference<>(0.0);
    private final AtomicReference<Double> swerveSpeed = new AtomicReference<>(0.0);

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel    ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     * @param zeroTicks            ticks when angle = 0
     */
    public SwerveModule(int driveMotorChannel, int steeringMotorChannel, double zeroTicks) {
        this.zeroTicks = zeroTicks;

        //Steer Motor
        this.steeringMotor = new WPI_TalonSRX(steeringMotorChannel);
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition, TM_MM_PID, TM_MM_CONFIG, steeringMotor);
        steeringMotor.setSensorPhase(false);
        steeringMotor.setInverted(true);
        StatusFrameDemolisher.demolishStatusFrames(steeringMotor, false);

        //Drive Motor
        this.driveMotor = new WPI_TalonFX(driveMotorChannel, MotorIDs.CANFD_NAME);
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition, DM_MM_PID, DM_MM_CONFIG, driveMotor);
        driveMotor.configAllowableClosedloopError(0, 5);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 20);
        driveMotor.setNeutralMode(NeutralMode.Brake);
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
        return swerveAngle.get();
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
        return swerveSpeed.get();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(SwerveUtil.clampAngle(getAngle())));
    }

    public void setDriveMotorVelocity(double metersPerSecond) {
        Logging.robotShuffleboard.getTab("Swerve").setEntry("target velocity " + driveMotor.getDeviceID(), metersPerSecond);
        driveMotor.set(TalonFXControlMode.Velocity, convertVelocityToTicksPer100ms(metersPerSecond));
    }

    public void setSteeringMotorAngle(double angleInRad) {
        Logging.robotShuffleboard.getTab("Swerve").setEntry("target Angle" + driveMotor.getDeviceID(), angleInRad);
        steeringMotor.set(ControlMode.MotionMagic, angleInRad);
    }

    public void updateSwerveInformation() {
        swerveAngle.set((steeringMotor.getSelectedSensorPosition() - zeroTicks) * STEER_MOTOR_TICK_TO_ANGLE);
        swerveSpeed.set(driveMotor.getSensorCollection().getIntegratedSensorVelocity() * DRIVE_MOTOR_TICK_TO_SPEED);
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
