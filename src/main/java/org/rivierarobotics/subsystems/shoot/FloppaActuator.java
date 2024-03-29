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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.lib.shuffleboard.RSTab;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.smartmotion.SparkMotionConfig;
import org.rivierarobotics.util.smartmotion.SparkSmartMotion;

import java.io.BufferedReader;
import java.io.FileReader;

public class FloppaActuator extends SubsystemBase {
    public static FloppaActuator INSTANCE;

    public static FloppaActuator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new FloppaActuator();
        }
        return INSTANCE;
    }

    public static double convertAngleToTicks(double angleInRads) {
        return angleInRads + ShooterConstant.ACTUATOR_ZERO_TICKS;
    }

    private final SparkSmartMotion actuatorController;
    private final CANSparkMax actuatorMotor;
    private double targetAngle = 0.0;

    public FloppaActuator() {
        this.actuatorMotor = new CANSparkMax(MotorIDs.SHOOTER_ANGLE, CANSparkMaxLowLevel.MotorType.kBrushless);
        PIDConfig actuatorConfig = new PIDConfig(0.15, 0.0, 0, 0.0);
        actuatorConfig.setRange(1);
        actuatorConfig.setTolerance(0.0);
        actuatorMotor.restoreFactoryDefaults();
        SparkMotionConfig sparkSmartMotionConfig = new SparkMotionConfig(
                true,
                ShooterConstant.MAX_ACTUATOR_VELOCITY, ShooterConstant.MAX_ACTUATOR_ACCELERATION,
                2, -ShooterConstant.MAX_ACTUATOR_VELOCITY,
                ShooterConstant.TIMEOUTMS, 10
        );
        RSTab tuningTab = Logging.robotShuffleboard.getTab("actuator tuning");
        this.actuatorController = new SparkSmartMotion(actuatorMotor, actuatorConfig, sparkSmartMotionConfig, tuningTab);
        this.actuatorMotor.getEncoder().setPositionConversionFactor(1);
        this.actuatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ShooterConstant.MAX_ACTUATOR_TICKS);
        this.actuatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ShooterConstant.MIN_ACTUATOR_TICKS);
        this.actuatorMotor.setSmartCurrentLimit(30);
        this.actuatorMotor.setSecondaryCurrentLimit(30);

        this.actuatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        this.actuatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 1000);
        this.actuatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        this.actuatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 1000);

        try (var br = new BufferedReader(new FileReader(Filesystem.getOperatingDirectory().getPath() + "/temp.txt"))) {
            actuatorMotor.getEncoder().setPosition(Float.parseFloat(br.readLine()));
        } catch (Exception e) {
            // Ignore all read exceptions
        }
    }

    /**
     * Takes angle in radians setpoint should be in rotations and adjusted for gearing.
     */
    public void setFloppaAngle(double angle) {
        this.targetAngle = angle;
        var setpoint = convertAngleToTicks(angle);
        setpoint = MathUtil.clamp(setpoint, ShooterConstant.MIN_ACTUATOR_TICKS, ShooterConstant.MAX_ACTUATOR_TICKS);
        actuatorController.getPidController().setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    public boolean floppasWithinTolerance(double tolerance) {
        return org.rivierarobotics.lib.MathUtil.isWithinTolerance(getAngle(), this.targetAngle, tolerance);
    }

    /**
     * Returns angle without gearing values possibly in rotations.
     */
    public double getAngle() {
        return (actuatorMotor.getEncoder().getPosition() - ShooterConstant.ACTUATOR_ZERO_TICKS);
    }

    public double getTicks() {
        return actuatorMotor.getEncoder().getPosition();
    }

    public void setVoltage(double voltage) {
        // Manual floppa control soft limits, autoaim has its own thing
        this.actuatorMotor.setVoltage(voltage);
    }
}
