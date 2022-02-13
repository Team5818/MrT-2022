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
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.statespace.PositionStateSpaceModel;
import org.rivierarobotics.util.statespace.SystemIdentification;

import java.util.EnumMap;

public class Climb extends SubsystemBase {

    public static Climb getInstance() {
        if (climb == null) {
            climb = new Climb();
        }
        return climb;
    }

    private static Climb climb;
    private final Compressor compressor;
    private static final double MAX_FORWARD_LIMIT = 924412;
    private static final double MAX_REVERSE_LIMIT = -927054;
    private static final double LOW_RADIANS = -2.37;
    private static final double MID_RADIANS = 2.64;
    private static final double HIGH_RADIANS = -4.88;

    //TODO: Find Value
    private static final int ENCODER_RESOLUTION = 2048;
    private static final double MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION;
    private static final double GEARING = 1 / 450.0;

    private final WPI_TalonFX climbMotor;
    private final DutyCycleEncoder encoder;
    private final PositionStateSpaceModel climbStateSpace;
    //TODO: SysID The climb using the middle bar of the climb
    private final SystemIdentification sysId = new SystemIdentification(0.0, 7.7154, 0.19185);

    private final EnumMap<Position, DigitalInput> climbSwitchesMap = new EnumMap<>(Position.class);
    private final EnumMap<Position, Piston> climbPistonsMap = new EnumMap<>(Position.class);

    private double ZERO_RADIANS = 0.0;

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

        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enabled();

        this.climbMotor = new WPI_TalonFX(MotorIDs.CLIMB_ROTATE);
        climbMotor.configForwardSoftLimitEnable(false);
        climbMotor.configForwardSoftLimitThreshold(MAX_FORWARD_LIMIT);
        climbMotor.configReverseSoftLimitEnable(false);
        climbMotor.configReverseSoftLimitThreshold(MAX_REVERSE_LIMIT);

        climbMotor.setNeutralMode(NeutralMode.Brake);
        this.encoder = new DutyCycleEncoder(6);
        this.encoder.setDistancePerRotation(2 * Math.PI);

        climbSwitchesMap.put(Position.LOW, new DigitalInput(2));
        climbSwitchesMap.put(Position.MID, new DigitalInput(1));
        climbSwitchesMap.put(Position.HIGH, new DigitalInput(0));

        climbPistonsMap.put(Position.LOW, new Piston(2));
        climbPistonsMap.put(Position.MID, new Piston(1));
        climbPistonsMap.put(Position.HIGH, new Piston(0));
        setCoast(false);
    }

    public void setPiston(Position climbModule, boolean isEngaged) {
        climbPistonsMap.get(climbModule).set(isEngaged);
    }

    public void openAllPistons() {
        for (Position p : climbPistonsMap.keySet()) {
            climbPistonsMap.get(p).set(false);
        }
    }

    public void setCoast(boolean coast) {
        if (coast) {
            climbMotor.setNeutralMode(NeutralMode.Coast);
        }
        else {
            climbMotor.setNeutralMode(NeutralMode.Brake);
        }

    }

    public void setOffset() {
        ZERO_RADIANS = encoder.getDistance();
    }

    public boolean isSwitchSet(Position climbModule) {
        return !climbSwitchesMap.get(climbModule).get();
    }

    public boolean isPistonSet(Position climbModule) {
        return !climbPistonsMap.get(climbModule).getState();
    }

    public void setPosition(double radians) {
        climbStateSpace.setPosition(radians);
    }

    public void setVoltage(double voltage) {
        if(Math.abs(getAngle()) > 5.5 && Math.signum(-voltage) == Math.signum(getAngle())) {
            climbMotor.setVoltage(0);
            return;
        }
        climbMotor.setVoltage(voltage);
    }

    public double getAngle() {
        return encoder.getDistance() - ZERO_RADIANS;
    }

    public double getRawTicks() {
        return climbMotor.getSelectedSensorPosition();
    }

    public enum Position {
        LOW(LOW_RADIANS),
        MID(MID_RADIANS),
        HIGH(HIGH_RADIANS);

        public final double locationRadians;

        Position(double rads) {
            this.locationRadians = rads;
        }
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
