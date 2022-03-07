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


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.statespace.PositionStateSpaceModel;
import org.rivierarobotics.util.statespace.SystemIdentification;
import org.rivierarobotics.util.statespace.VelocityStateSpaceModel;

public class Hood extends SubsystemBase {

    public static Hood getInstance() {
        if (hood == null) {
            hood = new Hood();
        }
        return hood;
    }

    private static Hood hood;
    private double angle;
    private double speed = 0;
    private final WPI_TalonFX leftFlywheel;
    private final WPI_TalonFX rightFlywheel;
    private final CANSparkMax elevation;
    //private final DutyCycleEncoder encoder;
    private final double fireMaxVoltage = 5;
    private final double aimMaxVoltage = 5;

    private PositionStateSpaceModel aimStateSpace;
    private VelocityStateSpaceModel rightSS;
    private VelocityStateSpaceModel leftSS;
    private SystemIdentification aimSysId = new SystemIdentification(0.01, 0.01, 0.01);
    private SystemIdentification leftSysId = new SystemIdentification(0.0, 0.001, 0.001);
    private SystemIdentification rightSysId = new SystemIdentification(0.0, 0.001, 0.001);

    public Hood() {
        this.elevation = new CANSparkMax(MotorIDs.SHOOTER_ANGLE, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.leftFlywheel = new WPI_TalonFX(MotorIDs.SHOOTER_LEFT);
        this.rightFlywheel = new WPI_TalonFX(MotorIDs.SHOOTER_RIGHT);
        //this.encoder = new DutyCycleEncoder(0);
       // this.encoder.setDistancePerRotation(2 * Math.PI);
        this.angle = getAngle();
        //TODO: all of this sysid
        this.rightSS = new VelocityStateSpaceModel(
                rightSysId,
                0.01,
                0.01,
                0.01,
                0.1,
                fireMaxVoltage

        );
        this.leftSS = new VelocityStateSpaceModel(
                leftSysId,
                0.01,
                0.01,
                0.01,
                0.1,
                fireMaxVoltage

        );
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
        leftFlywheel.setInverted(true);
        leftFlywheel.setStatusFramePeriod(10, 20);
        leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        rightFlywheel.setStatusFramePeriod(10, 20);
        rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

    }

    public double getAngle() {
        return 2.0;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
        this.rightSS.setVelocity(speed);
        this.leftSS.setVelocity(speed);
    }

    public void setAngle(double radians) {
        aimStateSpace.setPosition(radians);
    }

    @Override
    public void periodic() {
        //double rightVoltage = rightSS.getAppliedVoltage(rightFlywheel.getSensorCollection().getIntegratedSensorVelocity());
        //double leftVoltage = leftSS.getAppliedVoltage(leftFlywheel.getSensorCollection().getIntegratedSensorVelocity());
        double aimVoltage = aimStateSpace.getAppliedVoltage(getAngle());
        elevation.setVoltage(aimVoltage);
        rightFlywheel.setVoltage(speed);
        leftFlywheel.setVoltage(speed);
    }
}
