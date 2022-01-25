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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
    private static final double MAX_FORWARD_LIMIT = -5232;
    private static final double MAX_REVERSE_LIMIT = -748537;
    private static final double LOW_TICKS = -2.24;
    private static final double MID_TICKS = 1;
    private static final double HIGH_TICKS = 2;

    //TODO: Find Value
    private static final int ENCODER_RESOLUTION = 2048;
    private static final double MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION;
    private static final double GEARING = 1 / 450.0;

    private final WPI_TalonFX climbMotor;
    private final PositionStateSpaceModel climbStateSpace;
    //TODO: SysID The climb using the middle bar of the climb
    private final SystemIdentification sysId = new SystemIdentification(0.43863, 7.7154, 0.19185);

    private final EnumMap<Position, DigitalInput> climbSwitchesMap = new EnumMap<>(Position.class);
    private final EnumMap<Position, Piston> climbPistonsMap = new EnumMap<>(Position.class);

    private Climb() {
        this.climbStateSpace = new PositionStateSpaceModel(
                sysId,
                0.1,
                0.01,
                0.01,
                0.01,
                0.01,
                0.15,
                6
        );

        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enabled();

        this.climbMotor = new WPI_TalonFX(MotorIDs.CLIMB_ROTATE);
        climbMotor.configForwardSoftLimitEnable(true);
        climbMotor.configForwardSoftLimitThreshold(MAX_FORWARD_LIMIT);
        climbMotor.configReverseSoftLimitEnable(true);
        climbMotor.configReverseSoftLimitThreshold(MAX_REVERSE_LIMIT);

        climbSwitchesMap.put(Position.LOW, new DigitalInput(2));
        climbSwitchesMap.put(Position.MID, new DigitalInput(1));
        climbSwitchesMap.put(Position.HIGH, new DigitalInput(0));

        climbPistonsMap.put(Position.LOW, new Piston(2));
        climbPistonsMap.put(Position.MID, new Piston(1));
        climbPistonsMap.put(Position.HIGH, new Piston(0));
    }

    public void setPiston(Position climbModule, boolean isOpen) {
        climbPistonsMap.get(climbModule).set(isOpen);
    }

    public void openAllPistons() {
        for (Position p : climbPistonsMap.keySet()) {
            climbPistonsMap.get(p).set(true);
        }
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
        climbMotor.setVoltage(voltage);
    }

    public double getAngle() {
        return climbMotor.getSensorCollection().getIntegratedSensorPosition() * GEARING * MOTOR_TICK_TO_ANGLE;
    }

    public enum Position {
        LOW(LOW_TICKS),
        MID(MID_TICKS),
        HIGH(HIGH_TICKS);

        public final double locationTicks;

        Position(double ticks) {
            this.locationTicks = ticks;
        }
    }

    public void followStateSpace() {
        var climbVoltage = climbStateSpace.getAppliedVoltage(getAngle());
        setVoltage(climbVoltage);
    }

    @Override
    public void periodic() {
        Logging.robotShuffleboard.getTab("Climb").setEntry("Compressor Enabled", compressor.enabled());
        Logging.robotShuffleboard.getTab("Climb").setEntry("Compressor Pressure", compressor.getPressure());

    }
}
