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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.appjack.MechLogger;
import org.rivierarobotics.util.statespace.SystemIdentification;

public class Intake extends SubsystemBase {
    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }

    private static Intake intake;
    private final SystemIdentification sysId = new SystemIdentification(0.01, 0.01, 0.01);
    private MechLogger logger;

    //the 3 rollers that intake the ball
    private static TalonSRX roller1;
    private static TalonSRX roller2;
    private static TalonSRX roller3;


    private Intake(){
        roller1 = new TalonSRX(0);
        roller2 = new TalonSRX(1);
        roller3 = new TalonSRX(2);
    }

    public void setVoltage(double v){
        //sets voltage
        logger.powerChange(v);

        roller1.set(ControlMode.Velocity, v);
        roller2.set(ControlMode.Velocity, v);
        roller3.set(ControlMode.Velocity, v);
    }

}
