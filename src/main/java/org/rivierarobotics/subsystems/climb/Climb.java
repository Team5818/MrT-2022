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

package org.rivierarobotics.subsystems.climb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.lib.MotionMagicConfig;
import org.rivierarobotics.lib.MotorUtil;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.StatusFrameDemolisher;

import java.util.ArrayList;

public class Climb extends SubsystemBase {
    private static Climb INSTANCE;

    public static Climb getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Climb();
        }
        return INSTANCE;
    }

    // All of these values are bs change them later
    public static final double KP = 0.2;
    private static final MotionMagicConfig CM_MM_CONFIG = new MotionMagicConfig(
            new ArrayList<>(), true, ClimbConstants.MAX_CLIMB_VELOCITY, ClimbConstants.MAX_CLIMB_ACCELERATION,
            100, 0, ClimbConstants.TIMEOUT_MS, 10
    );
    private static final PIDConfig CM_MM_PID = new PIDConfig(KP, 0, 0, 0);
    private static final double RANGE = 467626;
    private static final double ABSOLUTE_ZERO_RADIANS = 0;

    private final WPI_TalonFX climbMaster;
    private final WPI_TalonFX climbFollower;
    private final DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(MotorIDs.CLIMB_ENCODER);
    private boolean play = true;
    public double zeroTicks;

    private Climb() {
        this.climbMaster = new WPI_TalonFX(MotorIDs.CLIMB_ROTATE_A, MotorIDs.CANFD_NAME);
        this.climbFollower = new WPI_TalonFX(MotorIDs.CLIMB_ROTATE_B, MotorIDs.CANFD_NAME);
        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor, CM_MM_PID, CM_MM_CONFIG, climbMaster);
        climbFollower.follow(climbMaster);
        setCoast(false);
        climbMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        climbMaster.setInverted(true);
        climbFollower.setInverted(true);
        climbMaster.configFeedbackNotContinuous(true, ClimbConstants.TIMEOUT_MS);
        climbMaster.setSensorPhase(false);
        climbFollower.configFeedbackNotContinuous(true, ClimbConstants.TIMEOUT_MS);
        climbFollower.setSensorPhase(false);
        climbMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 35, 2));
        climbFollower.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 35, 2));
        dutyCycleEncoder.setDistancePerRotation(Math.PI * 2);

        StatusFrameDemolisher.demolishStatusFrames(climbMaster, false);
        StatusFrameDemolisher.demolishStatusFrames(climbFollower, true);
        double tickAdjust = climbMaster.getSelectedSensorPosition();
        this.zeroTicks = tickAdjust;
        this.climbMaster.configForwardSoftLimitEnable(true);
        climbMaster.configReverseSoftLimitEnable(true);
        climbMaster.configReverseSoftLimitThreshold(tickAdjust - RANGE);
        climbMaster.configForwardSoftLimitThreshold(tickAdjust + RANGE);
    }

    public void setCoast(boolean coast) {
        if (coast) {
            climbMaster.setNeutralMode(NeutralMode.Coast);
            climbFollower.setNeutralMode(NeutralMode.Coast);
        } else {
            climbMaster.setNeutralMode(NeutralMode.Brake);
            climbFollower.setNeutralMode(NeutralMode.Brake);
        }
    }

    public double getDutyCyclePose() {
        return MathUtil.wrapToCircle(dutyCycleEncoder.getDistance() - ClimbConstants.ABSOLUTE_ZERO, Math.PI * 2);
    }

    public void resetZeros(boolean absolute) {
        double tickAdjust;
        if (absolute) {
            // You must replace the Absolute Zero first to make this work
            tickAdjust = climbMaster.getSelectedSensorPosition() + (getDutyCyclePose() - ABSOLUTE_ZERO_RADIANS) * ClimbConstants.MOTOR_ANGLE_TO_TICK;
        } else {
            tickAdjust = climbMaster.getSelectedSensorPosition();
        }
        this.zeroTicks = tickAdjust;
        climbMaster.configForwardSoftLimitEnable(true);
        climbMaster.configReverseSoftLimitEnable(true);
        climbMaster.configReverseSoftLimitThreshold(tickAdjust - RANGE);
        climbMaster.configForwardSoftLimitThreshold(tickAdjust + RANGE);
    }

    public void setPosition(double radians) {
        climbMaster.set(ControlMode.MotionMagic, radians * ClimbConstants.MOTOR_ANGLE_TO_TICK + zeroTicks);
    }

    public void setPlay(boolean play) {
        this.play = play;
    }

    public boolean getPlay() {
        return play;
    }

    public void setVoltage(double voltage) {
        climbMaster.setVoltage(play ? voltage : 0);
    }

    public double getAngle() {
        return (climbMaster.getSelectedSensorPosition() - zeroTicks) * ClimbConstants.MOTOR_TICK_TO_ANGLE;
    }

    /** For use clearing the saved MM angle. */
    public void killController() {
        climbMaster.set(ControlMode.Disabled, 0);
    }

    public double getRawAngle() {
        return climbMaster.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return climbMaster.getSelectedSensorVelocity();
    }
}
