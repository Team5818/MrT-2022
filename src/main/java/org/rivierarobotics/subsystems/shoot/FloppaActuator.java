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

package org.rivierarobotics.subsystems.shoot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.lib.PIDConfig;
import org.rivierarobotics.lib.shuffleboard.RSTab;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.smartmotion.SparkMotionConfig;
import org.rivierarobotics.util.smartmotion.SparkSmartMotion;


public class FloppaActuator extends SubsystemBase {
    public static FloppaActuator floppasActuator;
    public static FloppaActuator getInstance(){
        if (floppasActuator == null) {
            floppasActuator = new FloppaActuator();
        }
        return floppasActuator;
    }

    private final PIDConfig actuatorConfig;
    private final SparkMotionConfig sparkSmartMotionConfig;
    private final SparkSmartMotion actuatorController;
    private final RSTab tunningTab;
    private final CANSparkMax actuatorMotor;

    public FloppaActuator() {
        this.actuatorMotor = new CANSparkMax(MotorIDs.SHOOTER_ANGLE, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.actuatorConfig = new PIDConfig(0.1, 0, 0,0);
        this.sparkSmartMotionConfig = new SparkMotionConfig(
                true,
                ShooterConstant.MAX_ACTUATOR_VELOCITY, ShooterConstant.MAX_ACTUATOR_ACCELERATION,
                100, 2,
                ShooterConstant.TIMEOUTMS, 10
        );
        this.tunningTab = Logging.robotShuffleboard.getTab("actuator tuning");

        this.actuatorController = new SparkSmartMotion(actuatorMotor, actuatorConfig, sparkSmartMotionConfig, tunningTab);

//        this.actuatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ShooterConstant.MAX_ACTUATOR_LIMIT * );
//        this.actuatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ShooterConstant.MIN_ACTUATOR_LIMIT);

    }

    public static double convertAngleToTicks(double angleInRads) {
        return angleInRads * ShooterConstant.ACTUATOR_GEARING + ShooterConstant.ACTUATOR_ZERO_TICKS;
    }

    //takes angle in radians setpoint should be in rotations and adjusted for gearing

    public void setFloppaAngle(double angle){
        var setpoint = angle / (2 * Math.PI) * ShooterConstant.ACTUATOR_GEARING;
        actuatorController.getPidController().setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    //returns angle without gearing values possibly in rotations

    public double getAngle() {
        return (actuatorMotor.getEncoder().getPosition() - ShooterConstant.ACTUATOR_ZERO_TICKS) / ShooterConstant.ACTUATOR_GEARING;
    }

    public double getTicks(){
        return actuatorMotor.getEncoder().getPosition() / ShooterConstant.ACTUATOR_GEARING;
    }

    public void setVoltage(double voltage){
        this.actuatorMotor.setVoltage(voltage);
    }
}
