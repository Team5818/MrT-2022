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

import java.util.Objects;

// do not make this a subsystem please thank you
public class IntakeSensors {
    private static IntakeSensors INSTANCE;

    public static IntakeSensors getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSensors();
        }
        return INSTANCE;
    }

    //TODO remove these if they are unused
    private static final double COLOR_SENSOR_TOLERANCE = 0.1; //Color sensor's values can be 10% off of what the ideal values are.
    private static final String TESTING_ALLIANCE_COLOR = "red";

    private final ColorSensorV3 colorSensor;
    private final AnalogInput distanceSensor;
    private final AnalogInput distanceSensor2;

    public IntakeSensors() {
        this.colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        this.distanceSensor = new AnalogInput(3);
        this.distanceSensor2 = new AnalogInput(0);
    }

    public boolean colorSensorHasBall() {
        return Math.abs(colorSensor.getProximity()) > 170;
    }

    public boolean distanceSensorHasBall() {
        //TODO remove comment if unused
//        return (Math.abs(distanceSensor.getValue()) < 2000) || (Math.abs(distanceSensor2.getValue()) < 2000);
        return (Math.abs(distanceSensor.getValue()) < 2000);
    }

    public ColorSensorV3 getColorSensor() {
        return this.colorSensor;
    }

    public boolean isTeamBall() {
        //TODO remove comment if unused
//        return this.testingAllianceColor == getBallColor();
        var bc = getBallColor();

        if (bc.equals("no ball")) {
            return true;
        }

        //TODO Objects.equals(bc, "blue") --> bc.equals("blue") (more clear)
        //TODO simplify this logic down. At present it could be a big return-boolean but I feel like the basic logic is:
        // if station == color, true OR invalid == red, true
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue && Objects.equals(bc, "blue")) {
            return true;
        } else {
            return (DriverStation.getAlliance() == DriverStation.Alliance.Red || DriverStation.getAlliance() == DriverStation.Alliance.Invalid) && Objects.equals(bc, "red");
        }
    }

    public String getBallColor() {
        Color color = colorSensor.getColor();
        if (color.red > 0.35) {
            return "red";
        } else if (color.blue > 0.35) {
            return "blue";
        } else {
            return "no ball";
        }
    }

    public boolean canCollect() {
        //TODO remove comment if unused
        //return !distanceSensorHasBall();
        return !colorSensorHasBall() || !distanceSensorHasBall();
    }
}
