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
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    //private static final double zero_ticks = -0.3;
    private final Compressor compressor;
    private static final double MAX_FORWARD_LIMIT = 823742;
    private static final double MAX_REVERSE_LIMIT = -2255;
    private static final double gearing = 1 / ((54.0/12.0) * 100);
    private final WPI_TalonFX climbMotor = new WPI_TalonFX(MotorIDs.CLIMB_ROTATE);
    private final PositionStateSpaceModel climbStateSpace;
    //CHANGE THESE VALUES
    private final SystemIdentification sysId = new SystemIdentification(0.01, 0.01, 0.01);


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
        climbMotor.configForwardSoftLimitEnable(true);
        climbMotor.getSensorCollection().setIntegratedSensorPosition(0, 10);
        climbMotor.configForwardSoftLimitThreshold(MAX_FORWARD_LIMIT);
        climbMotor.configReverseSoftLimitEnable(true);
        climbMotor.configReverseSoftLimitThreshold(MAX_REVERSE_LIMIT);

    }

    public void setPosition(double ticks) {
        climbStateSpace.setPosition(ticks);
    }

    public void setVoltage(double voltage) {
        climbMotor.setVoltage(voltage);
    }

    public double getAngle() {
        return climbMotor.getSensorCollection().getIntegratedSensorPosition() * gearing / 2048;
    }

    @Override
    public void periodic() {
        Logging.robotShuffleboard.getTab("Climb").setEntry("Compressor Enabled", compressor.enabled());
        Logging.robotShuffleboard.getTab("Climb").setEntry("Compressor Pressure", compressor.getPressure());
        //var climbVoltage = climbStateSpace.getAppliedVoltage(getAngle());
        //setVoltage(climbVoltage);
    }
}
