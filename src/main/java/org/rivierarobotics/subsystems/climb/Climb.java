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
    private static final double STEER_MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION;
    private static final double GEARING = 1 / 450;

    private final WPI_TalonFX climbMotor;
    private final PositionStateSpaceModel climbStateSpace;
    //TODO: SysID The climb using the middle bar of the climb
    private final SystemIdentification sysId = new SystemIdentification(0.01, 0.01, 0.01);

    private static final DigitalInput[] climbSwitches = new DigitalInput[3];

    private static final Piston[] climbPistons = new Piston[3];

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

        climbPistons[0] = new Piston(0);
        climbPistons[1] = new Piston(1);
        climbPistons[2] = new Piston(2);
    }

    public void setPiston(ClimbModule climbModule, boolean isOpen) {
        climbModule.piston.set(isOpen);
    }

    public void openAllPistons() {
        for(int i = 0 ; i < climbPistons.length; i++){
           climbPistons[i].set(true);
        }
    }

    //TODO: Implement method using your switches stored at the class level
    public boolean isSwitchSet(ClimbModule climbModule) {
        return climbModule.sw.get();
    }

    public void setPosition(double radians) {
        climbStateSpace.setPosition(radians);
    }

    public void setVoltage(double voltage) {
        climbMotor.setVoltage(voltage);
    }

    public double getAngle() {
        return climbMotor.getSensorCollection().getIntegratedSensorPosition() * GEARING / ENCODER_RESOLUTION * STEER_MOTOR_TICK_TO_ANGLE;
    }

//    public enum ClimbPistons {
//
//        LOW(climbPistons[0]), MID(climbPistons[1]), HIGH(climbPistons[2]);
//        private final Piston id;
//
//        ClimbPistons(Piston id) {
//            this.id = id;
//        }
//
//        public boolean getState(){
//            return id.getState();
//        }
//
//        public Piston getPiston(){
//            return id;
//        }
//
//    }
//
//    public enum ClimbSwitches {
//        LOW(climbSwitches[0]),
//        MID(climbSwitches[1]),
//        HIGH(climbSwitches[2]);
//
//        public final DigitalInput swi;
//
//        ClimbSwitches(DigitalInput swi){
//            this.swi = swi;
//        }
//
//        public boolean isOpen(){
//            return !swi.get();
//        }
//
//    }

    public enum ClimbModule {
        LOW(climbSwitches[0], climbPistons[0], LOW_TICKS),
        MID(climbSwitches[1], climbPistons[1], MID_TICKS),
        HIGH(climbSwitches[2], climbPistons[2], HIGH_TICKS);

        public final DigitalInput sw;
        public final Piston piston;
        public final double ticks;

        ClimbModule(DigitalInput sw, Piston piston, double ticks){
            this.sw = sw;
            this.piston = piston;
            this.ticks = ticks;
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
