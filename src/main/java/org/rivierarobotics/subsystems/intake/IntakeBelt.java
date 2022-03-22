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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.subsystems.MotorIDs;

public class IntakeBelt extends SubsystemBase {
    private static IntakeBelt intakeBelt;
    public static IntakeBelt getInstance() {
        if(intakeBelt == null) {
            intakeBelt = new IntakeBelt();
        }
        return intakeBelt;
    }

    private final WPI_TalonSRX beltMotor;
    private final WPI_TalonSRX miniWheelMotor;

    public IntakeBelt() {
        beltMotor = new WPI_TalonSRX(MotorIDs.COLLECT_BELTS);
        this.miniWheelMotor = new WPI_TalonSRX(MotorIDs.COLLECT_WHEEL);
    }

    public void setBeltVoltage(double v) {
        beltMotor.setVoltage(v);
    }

    public void setMiniWheelMotorVoltage(double v) {
        miniWheelMotor.setVoltage(v);
    }
}
