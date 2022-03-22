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

package org.rivierarobotics.subsystems.intake;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

// do not make this a subsystem please thank you
public class IntakeSensors {

    private final static double COLOR_SENSOR_TOLERANCE = 0.1; //Color sensor's values can be 10% off of what the ideal values are.
    private final String testingAllianceColor = "red";

    private static IntakeSensors intakeSensors;
    public static IntakeSensors getInstance() {
        if(intakeSensors == null) {
            intakeSensors = new IntakeSensors();
        }
        return intakeSensors;
    }

    private final ColorSensorV3 colorSensorV3;
    private final AnalogInput distanceSensor;
    private final AnalogInput distanceSensor2;
    public IntakeSensors() {
        this.colorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);
        this.distanceSensor = new AnalogInput(3);
        this.distanceSensor2 = new AnalogInput(0);
    }

    public boolean colorSensorHasBall(){
        return Math.abs(colorSensorV3.getProximity()) > 170;
    }

    public boolean distanceSensorHasBall(){
        return (Math.abs(distanceSensor.getValue()) < 2000) || (Math.abs(distanceSensor2.getValue()) < 2000);
    }

    public ColorSensorV3 getColorSensorV3(){
        return this.colorSensorV3;
    }

    public boolean isTeamBall(){
        return this.testingAllianceColor == getBallColor();
//        return (DriverStation.getAlliance() == DriverStation.Alliance.Blue && getBallColor() == "blue") || (DriverStation.getAlliance() == DriverStation.Alliance.Red && getBallColor() == "red");
    }

    public String getBallColor() {
        Color color = colorSensorV3.getColor();
        if(color.red > 0.35) {
            return "red";
        } else if(color.blue > 0.35) {
            return "blue";
        } else {
            return "no ball";
        }
    }

    public boolean canCollect() {
        //return !distanceSensorHasBall();
        return !colorSensorHasBall() || !distanceSensorHasBall();
    }
}
