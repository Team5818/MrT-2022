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
    private final VelocityStateSpaceModel driveController;
    private final TalonSRX motor;
    private final boolean setDriveEnabled = false;

    //TODO: Extract ID's into MotorID's class
    public Intake() {
        // Figure out constants later
        this.p1 = new Piston(0);
        this.p2 = new Piston(1);
        this.motor = new TalonSRX(0);
        //TODO: Get rid of State Space stuff. This will be a setPower() or setVoltage() component. We don't need to actively manage its speed
        this.driveController = new VelocityStateSpaceModel(
                new SystemIdentification(0.01, 0.01, 0.01),
                0.,
                0.,
                0.,
                0.,
                0.
        );
    }


    public void setVelocity(double radPerSecond) {
        // Don't know what to do here.
        driveController.setVelocity(radPerSecond);
    }

    public void setIntakeState(boolean in) {
        p1.set(in);
        p2.set(in);
    }

    //TODO: Remove all state space stuff.
    @Override
    public void periodic() {
        if(!setDriveEnabled) return;
        var driveVoltage = driveController.getAppliedVoltage(1);

        //setDriveMotorVoltage(driveVoltage);
    }

}
