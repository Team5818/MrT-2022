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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.subsystems.climb.Piston;

public class Intake extends SubsystemBase {
    private static Intake intake;

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }

    //    private final Piston p1;
//    private final Piston p2;
    private final CANSparkMax csm;
    private final WPI_TalonSRX tsrx;
    private final boolean setDriveEnabled = false;

    private double intakeVoltage = 0;
    private double beltVoltage = 0;
    private boolean isDeployed = false;
    private int isPositive = 0;


    //TODO: Extract ID's into MotorID's class
    public Intake() {
        // Figure out constants later
//        p1 = new Piston(20);
//        p2 = new Piston(21);
        csm = new CANSparkMax(MotorIDs.COLLECT_INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);
        tsrx = new WPI_TalonSRX(MotorIDs.COLLECT_BELTS);
    }

    public void setIntakeState(boolean deploy) {
//        p1.set(deploy);
//        p2.set(deploy);
        this.isDeployed = deploy;
    }

//    public boolean getIntakeState() {
//        return p1.getState();
//    }

    public double getIntakeVoltage() {
        return this.intakeVoltage;
    }

    public double getBeltVoltage() {
        return this.beltVoltage;
    }

    public void setVoltages(double beltVoltage, double intakeVoltage) {
        this.beltVoltage = beltVoltage;
        tsrx.setVoltage(beltVoltage);

        this.intakeVoltage = intakeVoltage;
        csm.setVoltage(intakeVoltage);
    }

    public void setIsPositive(int isPositive) {
        this.isPositive = isPositive;
    }

    public int getIsPositive() {
        return this.isPositive;
    }
}