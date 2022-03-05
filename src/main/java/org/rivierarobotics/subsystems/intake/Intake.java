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

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.subsystems.climb.Piston;
import org.rivierarobotics.util.statespace.SystemIdentification;
import org.rivierarobotics.util.statespace.VelocityStateSpaceModel;

public class Intake extends SubsystemBase {
    private static Intake intake;

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }

    private final Piston p1;
    private final Piston p2;
    private final CANSparkMax motor;
    private final boolean setDriveEnabled = false;

    //TODO: Extract ID's into MotorID's class
    public Intake() {
        // Figure out constants later
        this.p1 = new Piston(0);
        this.p2 = new Piston(1);
        this.motor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public void setIntakeState(boolean deploy) {
        p1.set(deploy);
        p2.set(deploy);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

}
