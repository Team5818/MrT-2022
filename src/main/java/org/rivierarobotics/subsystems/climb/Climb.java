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
    private static final double MAX_FORWARD_LIMIT = 823742;
    private static final double MAX_REVERSE_LIMIT = -2255;
    private static final double LOW_TICKS = 0;
    private static final double MID_TICKS = 1;
    private static final double HIGH_TICKS = 2;

    private static final double WHEEL_RADIUS = 0.03915;
    //TODO: Find Value
    private static final int ENCODER_RESOLUTION = 2048;
    private static final double MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION;
    //TODO: add a decimal point to actually make this a double
    private static final double GEARING = 1 / 450.;

    private final WPI_TalonFX climbMotor;
    private final PositionStateSpaceModel climbStateSpace;
    //TODO: SysID The climb using the middle bar of the climb
    private final SystemIdentification sysId = new SystemIdentification(0.01, 0.01, 0.01);

    private static final DigitalInput[] climbSwitches = new DigitalInput[3];
    private static final Piston[] climbPistons = new Piston[3];
    private static final EnumMap<ClimbModule, DigitalInput> climbSwitchesMap = new EnumMap<ClimbModule, DigitalInput>(ClimbModule.class);
    private static final EnumMap<ClimbModule, Piston> climbPistonsMap = new EnumMap<ClimbModule, Piston>(ClimbModule.class);

    private Climb() {
        this.climbStateSpace = new PositionStateSpaceModel(
                sysId,
                0.01,
                0.01,
                0.01,
                0.01,
                0.01,
                0.01,
                0.01
        );

        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enabled();

        this.climbMotor = new WPI_TalonFX(MotorIDs.CLIMB_ROTATE);
        climbMotor.configForwardSoftLimitEnable(true);
        climbMotor.getSensorCollection().setIntegratedSensorPosition(0, 10);
        climbMotor.configForwardSoftLimitThreshold(MAX_FORWARD_LIMIT);
        climbMotor.configReverseSoftLimitEnable(true);
        climbMotor.configReverseSoftLimitThreshold(MAX_REVERSE_LIMIT);

        climbSwitches[0] = new DigitalInput(0);
        climbSwitches[1] = new DigitalInput(1);
        climbSwitches[2] = new DigitalInput(2);
        climbSwitchesMap.put(ClimbModule.LOW, climbSwitches[0]);
        climbSwitchesMap.put(ClimbModule.MID, climbSwitches[1]);
        climbSwitchesMap.put(ClimbModule.HIGH, climbSwitches[2]);


        climbPistons[0] = new Piston(0);
        climbPistons[1] = new Piston(1);
        climbPistons[2] = new Piston(2);
        climbPistonsMap.put(ClimbModule.LOW, climbPistons[0]);
        climbPistonsMap.put(ClimbModule.MID, climbPistons[1]);
        climbPistonsMap.put(ClimbModule.HIGH, climbPistons[2]);
    }

    public static EnumMap<ClimbModule, DigitalInput> getClimbSwitchesMap() {
        return climbSwitchesMap;
    }

    public static EnumMap<ClimbModule, Piston> getClimbPistonsMap() {
        return climbPistonsMap;
    }

    public void setPiston(ClimbModule climbModule, boolean isOpen) {
        climbPistonsMap.get(climbModule).set(isOpen);
    }

    public void openAllPistons() {
        for(int i = 0 ; i < climbPistons.length; i++){
           climbPistons[i].set(true);
        }
    }

    public boolean isSwitchSet(ClimbModule climbModule) {
        return climbSwitchesMap.get(climbModule).get();
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

    public enum ClimbModule {
        LOW(LOW_TICKS),
        MID(MID_TICKS),
        HIGH(HIGH_TICKS);

        public final double locationTicks;

        ClimbModule(double ticks){
            this.locationTicks=ticks;
        }

    }

    @Override
    public void periodic() {
        Logging.robotShuffleboard.getTab("Climb").setEntry("Compressor Enabled", compressor.enabled());
        Logging.robotShuffleboard.getTab("Climb").setEntry("Compressor Pressure", compressor.getPressure());
        //var climbVoltage = climbStateSpace.getAppliedVoltage(getAngle());
        //setVoltage(climbVoltage);
    }
}
