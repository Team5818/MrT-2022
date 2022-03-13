package org.rivierarobotics.util.smartmotion;/*
 * This file is part of 5818-lib, licensed under the GNU General Public License (GPLv3).
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import org.rivierarobotics.lib.MotionMagicConfig;
import org.rivierarobotics.lib.MotorUtil;
import org.rivierarobotics.lib.PIDConfig;

import java.util.ArrayList;
import java.util.List;

/**
 * Stores a configuration of Spark Motion related constants
 */
public class SparkMotionConfig extends MotionMagicConfig {
    private Double minVel;

    /**
     * Constructs a Spark Motion configuration with the specified constants.
     *
     * @param reset reset the controller to factory default if true.
     * @param maxVel maximum velocity of Motion Magic controller in ticks per 100ms.
     * @param maxAccel maximum acceleration of Spark Motion controller in ticks per 100ms.
     * @param integralZone constant for zone/range of integral term
     *                     in closed-loop error calculation.
     * @param minVel minimum velocity of Spark Motion controller in ticks per 100ms
     * @param timeoutMs timeout of all controller calls in milliseconds.
     * @param periodMs period of all status frame set calls in milliseconds.
     *
     * @since 0.3.0
     */
    public SparkMotionConfig(boolean reset,
                             Double maxVel, Double maxAccel,
                             Integer integralZone, double minVel,
                             int timeoutMs, int periodMs) {
        super(new ArrayList<>(), reset,maxVel,maxAccel,integralZone, 0, timeoutMs, periodMs);
        this.minVel = minVel;
    }

    public SparkMotionConfig(int timeoutMs, int periodMs) {
        this(false, 0.0, 0.0, 0, 0, timeoutMs, periodMs);
    }

    public SparkMotionConfig() {
        this(10, 10);
    }

    public Double getMinVel() {
        return minVel;
    }

    public void setMinVel(Double minVel) {
        this.minVel = minVel;
    }


}
