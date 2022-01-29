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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.util.statespace.PositionStateSpaceModel;
import org.rivierarobotics.util.statespace.SystemIdentification;
import org.rivierarobotics.util.statespace.VelocityStateSpaceModel;

public class SwerveModule extends SubsystemBase {
    private static final double WHEEL_RADIUS = 0.03915;
    private static final int ENCODER_RESOLUTION = 4096;
    private static final double STEER_MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION;
    private static final double GEARING = 11.0 / 40.0;

    private final double zeroTicks;
    private double currDriveVoltage = 0;
    private double currSteerVoltage = 0;
    private double targetVelocity = 0;

    private final CANSparkMax driveMotor;
    private final VelocityStateSpaceModel driveController;
    private final SystemIdentification dmSID = new SystemIdentification(0.12859, 5.0379, 0.03951);
    private final WPI_TalonSRX steeringMotor;
    private boolean setDriveEnabled = false;
    private final VelocityStateSpaceModel steerController;
    private final SystemIdentification tmSID = new SystemIdentification(0.43078, 0.93224, 1.4245);
    private final ProfiledPIDController turnMotorVelocityPID =
            new ProfiledPIDController(0.4, 0.0, 0.0, new TrapezoidProfile.Constraints(3,2));
    private Rotation2d targetRotation = new Rotation2d(0);
    private Rotation2d targetRotationClamped = new Rotation2d(0);


    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel    ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     * @param zeroTicks           ticks when angle = 0
     */
    public SwerveModule(int driveMotorChannel, int steeringMotorChannel, double zeroTicks, boolean driveInverted, boolean steeringInverted) {
        this.driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.steeringMotor = new WPI_TalonSRX(steeringMotorChannel);
        this.zeroTicks = zeroTicks;

        driveMotor.getEncoder().setPositionConversionFactor(GEARING * (2 * Math.PI * WHEEL_RADIUS));
        driveMotor.getEncoder().setVelocityConversionFactor(GEARING * (2 * Math.PI * WHEEL_RADIUS) / 60.0);
        driveMotor.getEncoder().setPosition(0);
        driveMotor.setInverted(driveInverted);

        steeringMotor.setInverted(steeringInverted);
        steeringMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, (int)(DriveTrain.STATE_SPACE_LOOP_TIME * 1000));

        this.driveController = new VelocityStateSpaceModel(
                dmSID, 0.1, 0.01,
                0.1, 4, 12, DriveTrain.STATE_SPACE_LOOP_TIME
        );
        this.driveController.setKsTolerance(0.05);

        this.steerController = new VelocityStateSpaceModel(
                tmSID, 0.4,
                0.01, 0.01,
                0.017, 12, DriveTrain.STATE_SPACE_LOOP_TIME
        );

        this.steerController.setVelocity(0);
        this.steerController.setKsTolerance(0.1);

        this.steeringMotor.configContinuousCurrentLimit(40);
        this.steeringMotor.configPeakCurrentLimit(40);
    }

    private double clampAngle(double angle) {
        double high = Math.PI;
        angle = MathUtil.wrapToCircle(angle, 2 * Math.PI);
        if (angle > high) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public double getAbsoluteAngle() {
        return clampAngle(getAngle());
    }

    public double getAngle() {
        steeringMotor.getSelectedSensorPosition();
        return (-steeringMotor.getSensorCollection().getPulseWidthPosition() - zeroTicks) * STEER_MOTOR_TICK_TO_ANGLE;
    }

    public double getSteeringMotorSpeed() {
        return (-(steeringMotor.getSensorCollection().getPulseWidthVelocity() * 10)) * STEER_MOTOR_TICK_TO_ANGLE;
    }

    public double getPosTicks() {
        return -steeringMotor.getSensorCollection().getPulseWidthPosition();
    }

    public double getDriveTicks() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getDriveVoltage() {
        return currDriveVoltage;
    }

    public double getSteerVoltage() {
        return currSteerVoltage;
    }

    public double getVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(clampAngle(getAngle())));
    }

    public void setDriveMotorVelocity(double metersPerSecond) {
        driveController.setVelocity(metersPerSecond);
    }

    public void setSteeringMotorAngle(double angleInRad) {
        turnMotorVelocityPID.setGoal(angleInRad);
    }

    public void setDriveMotorVoltage(double voltage) {
        this.currDriveVoltage = voltage;
        driveMotor.setVoltage(voltage);
    }

    public void setSteeringMotorVoltage(double voltage) {
        this.currSteerVoltage = voltage;
        steeringMotor.setVoltage(voltage);
    }

    public double getAngleDiff(double src, double target) {
        double diff = target - src;
        if (Math.abs(diff) <= Math.PI) {
            return diff;
        }

        if (diff > 0) {
            diff -= 2 * Math.PI;
        } else {
            diff += 2 * Math.PI;
        }

        return diff;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        SmartDashboard.putNumber(driveMotor.getDeviceId() + "", state.speedMetersPerSecond);
        //Update State-Space Controllers
        double targetSpeed = state.speedMetersPerSecond;
        double targetRotation = state.angle.getRadians();
        double currAng = getAngle();
        double clampedAng = clampAngle(getAngle());
        double posTarget = targetRotation + Math.PI;
        double negTarget = targetRotation - Math.PI;
        double diff;

        if (Math.abs(getAngleDiff(clampedAng, posTarget)) <= Math.abs(getAngleDiff(clampedAng, negTarget))) {
            diff = getAngleDiff(clampedAng, posTarget);
        } else {
            diff = getAngleDiff(clampedAng, negTarget);
        }

        if (Math.abs(getAngleDiff(clampedAng, targetRotation)) <= Math.abs(diff)) {
            diff = getAngleDiff(clampedAng, targetRotation);
        }

        double targetAng = currAng + diff;

        targetSpeed = state.speedMetersPerSecond;
        if (!MathUtil.isWithinTolerance(targetRotation, clampAngle(targetAng), 0.1)) {
            targetSpeed *= -1;
        }

        this.targetRotation = new Rotation2d(targetAng);
        this.targetRotationClamped = new Rotation2d(clampAngle(targetAng));
        this.targetVelocity = targetSpeed;

        setDriveMotorVelocity(targetSpeed);
        setSteeringMotorAngle(targetAng);
        setDriveEnabled = true;
    }

    public Rotation2d getTargetRotation() {
        return targetRotation;
    }

    public Rotation2d getTargetRotationClamped() {
        return targetRotationClamped;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getSteerMotorVel() {
        return getSteeringMotorSpeed();
    }

    public void testSetSpeed(double speed) {
        SmartDashboard.putNumber("called", speed);
        steerController.setVelocity(speed);
    }

    public void followControllers() {
        if(!setDriveEnabled) return;
        var driveVoltage = driveController.getAppliedVoltage(getVelocity());
        setDriveMotorVoltage(driveVoltage);

        var targetSpeed = MathUtil.isWithinTolerance(getAngle(), getTargetRotation().getRadians(), 0.05) ? 0 : turnMotorVelocityPID.calculate(getAngle());
        steerController.setVelocity(targetSpeed);
        SmartDashboard.putNumber("Target Speed", targetSpeed);
        SmartDashboard.putNumber("Vel Error", turnMotorVelocityPID.getVelocityError());
        SmartDashboard.putNumber("Position Error", turnMotorVelocityPID.getPositionError());
        SmartDashboard.putNumber("Goal", turnMotorVelocityPID.getGoal().position);
        var turnMotorVoltage = steerController.getAppliedVoltage(getSteeringMotorSpeed());
        //setSteeringMotorVoltage(turnMotorVoltage);
    }
}
