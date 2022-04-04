/*
 * This file is part of MrT-2022, licensed under the GNU General Public License (GPLv3).
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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.subsystems.MotorIDs;

public class IntakeRollers extends SubsystemBase {
    private static IntakeRollers INSTANCE;

    public static IntakeRollers getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeRollers();
        }
        return INSTANCE;
    }

    private final CANSparkMax rollerMotor;

    public IntakeRollers() {
        this.rollerMotor = new CANSparkMax(MotorIDs.COLLECT_INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);
        rollerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        rollerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 1000);
        rollerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 1000);
        rollerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 1000);
        rollerMotor.setSmartCurrentLimit(30);
        rollerMotor.setSecondaryCurrentLimit(30);
    }

    public void setRollerVoltage(double v) {
        rollerMotor.setVoltage(v);
    }
}
