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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.apache.commons.math3.analysis.function.Log;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.InterpolationTable;
import org.rivierarobotics.util.statespace.PositionStateSpaceModel;
import org.rivierarobotics.util.statespace.SystemIdentification;
import org.rivierarobotics.util.statespace.VelocityStateSpaceModel;

public class Floppas extends SubsystemBase {

    public static Floppas getInstance() {
        if (floppas == null) {
            floppas = new Floppas();
        }
        return floppas;
    }

    private static Floppas floppas;
    private double angle;
    private double speed = 0;
    private final WPI_TalonFX leftFlywheel;
    private final WPI_TalonFX rightFlywheel;
    private final CANSparkMax flopperMotor;
    private final DutyCycleEncoder floppaEncoder;
    private final double fireMaxVoltage = 12;
    private final double aimMaxVoltage = 9;
    private final double AIM_DOWNWARD_LIMIT = -3.71;
    private final double AIM_UPWARD_LIMIT = -2.0;
    private final double ZERO_ANGLE = -3.24;
    private boolean blockSS = false;
    private final double VEL_TO_RADS = (2 * Math.PI / 4096) * 10;
    private double targetV = 100;

    private PositionStateSpaceModel aimStateSpace;
    private VelocityStateSpaceModel rightSS;
    private VelocityStateSpaceModel leftSS;
    private SystemIdentification aimSysId = new SystemIdentification(0.0, 2.1, 0.04);
    private SystemIdentification leftSysId = new SystemIdentification(0, 0.017898 * 2.1, 0.000084794);
    private SystemIdentification rightSysId = new SystemIdentification(0, 0.017898 * 2.1, 0.000084794);
    private InterpolationTable intTable = new InterpolationTable();

    public void setBlockSS(boolean blockSS) {
        this.blockSS = blockSS;
    }

    public double getTargetV() {
        return targetV;
    }

    public void setTargetV(double targetV) {
        this.targetV = targetV;
    }

    public Floppas() {
        this.flopperMotor = new CANSparkMax(MotorIDs.SHOOTER_ANGLE, CANSparkMaxLowLevel.MotorType.kBrushless);
        flopperMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        this.leftFlywheel = new WPI_TalonFX(MotorIDs.SHOOTER_LEFT);
        this.rightFlywheel = new WPI_TalonFX(MotorIDs.SHOOTER_RIGHT);
        this.floppaEncoder = new DutyCycleEncoder(0);
        this.floppaEncoder.setDistancePerRotation(-2 * Math.PI);
        this.angle = getAngle();
        //TODO: all of this sysid
        this.rightSS = new VelocityStateSpaceModel(
                rightSysId,
                2,
                0.01,
                0.01,
                0.9,
                fireMaxVoltage

        );
        this.leftSS = new VelocityStateSpaceModel(
                leftSysId,
                2,
                0.01,
                0.01,
                0.9,
                fireMaxVoltage

        );
        this.aimStateSpace = new PositionStateSpaceModel(
                aimSysId,
                0.05,
                0.2,
                0.01,
                0.01,
                0.01,
                6,
                aimMaxVoltage

        );
        leftSS.setVelocity(0);
        rightSS.setVelocity(0);
        flopperMotor.setInverted(true);
        leftFlywheel.setInverted(false);
        leftFlywheel.setStatusFramePeriod(10, 20);
        leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        rightFlywheel.setStatusFramePeriod(10, 20);
        rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    }

    public enum ShooterLocations{
        LAUNCHPAD_A(0,0,0),
        LAUNCHPAD_B(1,1,0),
        LOW_GOAL(60,-3.927,0),
        FENDER(110,-1.9,0);

        public final double flyWheelSpeed;
        public final double floppaAngle;
        public final double driveAngle;

        ShooterLocations(double flyWheelSpeed, double floppaAngle, double driveAngle){
            this.flyWheelSpeed = flyWheelSpeed;
            this.floppaAngle = floppaAngle;
            this.driveAngle = driveAngle;
        }
    }


    public double getAngle() {
        return floppaEncoder.getDistance();
    }

    public void setSpeed(double speed) {
        this.speed = speed;
        this.rightSS.setVelocity(speed);
        this.leftSS.setVelocity(speed);
    }

    public void setShooterVoltage(double v) {
        rightFlywheel.setVoltage(v);
        leftFlywheel.setVoltage(-v);
    }

    public double getRightSpeed(){
        return rightFlywheel.getSensorCollection().getIntegratedSensorVelocity() * VEL_TO_RADS;
    }

    public double getLeftSpeed(){
        return leftFlywheel.getSensorCollection().getIntegratedSensorVelocity() * VEL_TO_RADS;
    }

    public double getTarget(boolean left) {
        if (left) {
            return leftSS.getTargetVelocity();
        } else {
            return rightSS.getTargetVelocity();
        }
    }

    public void setActuatorVoltage(double voltage) {
        if(floppaEncoder.getDistance() <= AIM_DOWNWARD_LIMIT && voltage < 0 || floppaEncoder.getDistance() >= AIM_UPWARD_LIMIT && voltage > 0) {
            flopperMotor.setVoltage(0);
            return;
        }

        flopperMotor.setVoltage(voltage);
    }

    public void setAngle(double radians) {
        var sb = Logging.robotShuffleboard;
        var limeLight = sb.getTab("LL");
        limeLight.setEntry("Hood Target", radians);
        aimStateSpace.setPosition(radians);
    }

    public void floppaStateSpaceControl() {
        var sb = Logging.robotShuffleboard;
        var limeLight = sb.getTab("LL");
        limeLight.setEntry("tpose", aimStateSpace.getTargetPosition());
        double aimVoltage = aimStateSpace.getAppliedVoltage(getAngle());
        limeLight.setEntry("AV", aimVoltage);

        setActuatorVoltage(aimVoltage);
    }

    @Override
    public void periodic() {
        double rightVoltage = rightSS.getAppliedVoltage(getRightSpeed());
        double leftVoltage = leftSS.getAppliedVoltage(-getLeftSpeed());
        if(blockSS) return;
        rightFlywheel.setVoltage(rightVoltage < 0 ? 0 : rightVoltage);
        leftFlywheel.setVoltage(leftVoltage < 0 ? 0 : -leftVoltage);
        var limeLight = Logging.robotShuffleboard.getTab("LL");
        limeLight.setEntry("rightV", rightVoltage);
        limeLight.setEntry("leftV", leftVoltage);

        SmartDashboard.putNumber("hood angle", getAngle());
        SmartDashboard.putNumber("right SS", rightSS.getTargetVelocity());
        SmartDashboard.putNumber("left SS", leftSS.getTargetVelocity());
    }
    public double getValueFromTable(double key){
        return intTable.getValue(key);
    }
}
