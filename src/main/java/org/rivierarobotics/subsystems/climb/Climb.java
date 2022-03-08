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

package org.rivierarobotics.subsystems.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.statespace.PositionStateSpaceModel;
import org.rivierarobotics.util.statespace.SystemIdentification;

public class Climb extends SubsystemBase {

    public static Climb getInstance() {
        if (climb == null) {
            climb = new Climb();
        }
        return climb;
    }

    private static Climb climb;
    private static final double MAX_FORWARD_LIMIT = 924412;
    private static final double MAX_REVERSE_LIMIT = -927054;
    private static final double LOW_RADIANS = -2.37;
    private static final double MID_RADIANS = 2.64;
    private static final double HIGH_RADIANS = -4.88;
    private static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    //TODO: Find Value using new climb once robot is built, these values are for the prototype.
    private static final int ENCODER_RESOLUTION = 2048;
    private static final double MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION;
    private static final double GEARING = 1 / 450.0;

    public enum Position {
        LOW(LOW_RADIANS, new Piston(3), new DigitalInput(2)),
        MID(MID_RADIANS, new Piston(4), new DigitalInput(1)),
        HIGH(HIGH_RADIANS, new Piston(5), new DigitalInput(0));

        public final double locationRadians;
        public final Piston piston;
        public final DigitalInput input;

        Position(double rads, Piston piston, DigitalInput input) {
            this.locationRadians = rads;
            this.piston = piston;
            this.input = input;
        }
    }

    private final WPI_TalonFX climbMotorA;
    private final DutyCycleEncoder encoder;
    private final PositionStateSpaceModel climbStateSpace;
    //TODO: SysID The climb using the middle bar of the climb once new climb is built, this works on cyclone
    private final SystemIdentification sysId = new SystemIdentification(0.0, 7.7154, 0.19185);
    private double zeroRadians = 0.0;

    private Climb() {
        this.climbStateSpace = new PositionStateSpaceModel(
                sysId,
                0.1,
                0.01,
                0.05,
                0.01,
                0.01,
                0.3,
                11
        );

        this.climbMotorA = new WPI_TalonFX(MotorIDs.CLIMB_ROTATE_A);
        climbMotorA.configForwardSoftLimitEnable(false);
        climbMotorA.configForwardSoftLimitThreshold(MAX_FORWARD_LIMIT);
        climbMotorA.configReverseSoftLimitEnable(false);
        climbMotorA.configReverseSoftLimitThreshold(MAX_REVERSE_LIMIT);

        climbMotorA.setNeutralMode(NeutralMode.Brake);
        this.encoder = new DutyCycleEncoder(6);
        this.encoder.setDistancePerRotation(2 * Math.PI);

        setCoast(false);
    }

    public void setPiston(Position climbModule, boolean isEngaged) {
        climbModule.piston.set(isEngaged);
    }

    public void openAllPistons() {
        Position.LOW.piston.set(false);
        Position.MID.piston.set(false);
        Position.HIGH.piston.set(false);
    }

    public void setCoast(boolean coast) {
        if (coast) {
            climbMotorA.setNeutralMode(NeutralMode.Coast);
        } else {
            climbMotorA.setNeutralMode(NeutralMode.Brake);
        }

    }

    public void setOffset() {
        this.zeroRadians = encoder.getDistance();
    }

    public boolean isSwitchSet(Position climbModule) {
        return !climbModule.input.get();
    }

    public boolean isPistonSet(Position climbModule) {
        return climbModule.piston.getState();
    }

    public void setPosition(double radians) {
        climbStateSpace.setPosition(radians);
    }

    public void setVoltage(double voltage) {
        if (Math.abs(getAngle()) > 5.5 && Math.signum(-voltage) == Math.signum(getAngle())) {
            climbMotorA.setVoltage(0);
            return;
        }
        climbMotorA.setVoltage(voltage);
    }

    public double getAngle() {
        return encoder.getDistance() - zeroRadians;
    }

    public double getRawTicks() {
        return climbMotorA.getSelectedSensorPosition();
    }

    public void followStateSpace() {
        var climbVoltage = climbStateSpace.getAppliedVoltage(getAngle());
        setVoltage(-climbVoltage);
    }

    @Override
    public void periodic() {
        Logging.robotShuffleboard.getTab("Climb").setEntry("Compressor Enabled", compressor.enabled());
        Logging.robotShuffleboard.getTab("Climb").setEntry("Compressor Pressure", compressor.getPressure());
        Logging.robotShuffleboard.getTab("Climb").setEntry("Climb Encoder", getAngle());
        Logging.robotShuffleboard.getTab("Climb").setEntry("Climb Offset", encoder.getFrequency());
    }
}