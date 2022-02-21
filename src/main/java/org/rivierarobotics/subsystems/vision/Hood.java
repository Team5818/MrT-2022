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

package org.rivierarobotics.subsystems.vision;

import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.subsystems.*;
import org.rivierarobotics.util.statespace.*;

public class Hood extends SubsystemBase {
    private static Hood hood;
    private double angle;
    private double speed = 0;
    private final WPI_TalonFX upper;
    private final WPI_TalonFX lower;
    private final CANSparkMax elevation;
    private final DutyCycleEncoder encoder;
    private final double fireMaxVoltage = 5;
    private final double aimMaxVoltage = 5;

    private PositionStateSpaceModel aimStateSpace;
    private SystemIdentification aimSysId = new SystemIdentification(0.0, 0.001, 0.001);


    public static Hood getInstance() {
        if (hood == null) {
            hood = new Hood();
        }
        return hood;
    }

    public Hood() {
        this.elevation = new CANSparkMax(MotorIDs.SHOOTER_ELEVATION, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.upper = new WPI_TalonFX(MotorIDs.UPPER_FIRING_WHEEL);
        this.lower = new WPI_TalonFX(MotorIDs.LOWER_FIRING_WHEEL);
        this.lower.setInverted(true);
        this.encoder = new DutyCycleEncoder(9);
        this.encoder.setDistancePerRotation(2 * Math.PI);
        this.angle = getAngle();
        this.aimStateSpace = new PositionStateSpaceModel(
                aimSysId,
                0.01,
                0.01,
                0.01,
                0.01,
                0.01,
                0.01,
                aimMaxVoltage

        );
    }

    public double getAngle() {
        return encoder.getDistance();
    }

    public void setSpeed(double speed) {
        this.speed = speed;
        if (this.speed > fireMaxVoltage) {
            this.speed = fireMaxVoltage;
        }
    }

    public void setAngle(double radians) {
        this.angle = radians;
    }

    @Override
    public void periodic() {
        upper.setVoltage(speed);
        lower.setVoltage(speed);
        var aimVoltage = aimStateSpace.getAppliedVoltage(getAngle());
        elevation.setVoltage(aimVoltage);
    }
}
