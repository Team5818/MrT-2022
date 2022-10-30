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

package org.rivierarobotics.subsystems.shoot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.lib.MotionMagicConfig;
import org.rivierarobotics.lib.MotorUtil;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.subsystems.MotorIDs;

import java.util.ArrayList;

public class FloppaFlywheels extends SubsystemBase {
    private static FloppaFlywheels INSTANCE;

    public static FloppaFlywheels getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new FloppaFlywheels();
        }
        return INSTANCE;
    }

    private static final PIDConfig FLYWHEEL_CONFIG_L = new PIDConfig(0.13, 0.001, 6.2, 0.047);
    private static final PIDConfig FLYWHEEL_CONFIG_R = new PIDConfig(0.13, 0.001, 6.2, 0.047);
    private static final MotionMagicConfig MOTION_MAGIC_CONFIG = new MotionMagicConfig(
            new ArrayList<>(), true,
            ShooterConstant.MAX_FLYWHEEL_VELOCITY, ShooterConstant.MAX_FLYWHEEL_ACCELERATION,
            500, 0, ShooterConstant.TIMEOUTMS, 10
    );

    public final WPI_TalonFX leftFlywheel;
    public final WPI_TalonFX rightFlywheel;

    private double targetVelocity = 8000;

    public FloppaFlywheels() {
        this.leftFlywheel = new WPI_TalonFX(MotorIDs.SHOOTER_LEFT, MotorIDs.CANFD_NAME);
        this.rightFlywheel = new WPI_TalonFX(MotorIDs.SHOOTER_RIGHT, MotorIDs.CANFD_NAME);
        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor, FLYWHEEL_CONFIG_L, MOTION_MAGIC_CONFIG, leftFlywheel);
        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor, FLYWHEEL_CONFIG_R, MOTION_MAGIC_CONFIG, rightFlywheel);
        leftFlywheel.configAllowableClosedloopError(0, 50);
        rightFlywheel.configAllowableClosedloopError(0, 50);
        leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftFlywheel.setInverted(true);
    }

    public void setFlywheelSpeed(double speed) {
        leftFlywheel.set(ControlMode.Velocity, speed);
        rightFlywheel.set(ControlMode.Velocity, speed);
    }

    public boolean flywheelsWithinTolerance(double tolerance) {
        boolean left = MathUtil.isWithinTolerance(getLeftFlywheelSpeed(), leftFlywheel.getClosedLoopTarget(), tolerance);
        boolean right = MathUtil.isWithinTolerance(getRightFlywheelSpeed(), rightFlywheel.getClosedLoopTarget(), tolerance);
        return left && right;
    }

    public double getLeftFlywheelSpeed() {
        return leftFlywheel.getSensorCollection().getIntegratedSensorVelocity();
    }

    public double getRightFlywheelSpeed() {
        return rightFlywheel.getSensorCollection().getIntegratedSensorVelocity();
    }

    public void setVoltage(double v) {
        leftFlywheel.setVoltage(v);
        rightFlywheel.setVoltage(v);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }
}
