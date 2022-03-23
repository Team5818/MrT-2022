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

package org.rivierarobotics.commands.basic.collect;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.intake.IntakeBelt;

//TODO I don't think you need this class. Just use: new SetBeltVoltage().withTimeout()
// and then you don't have to do any of the timing logic

// I would say keep the class because it's convenient.
// You can turn it into a sequential command that is just SetBeltVoltage() then new WaitCommand() then SetBeltVoltage() back to 0.
public class SetBeltVoltageWithTimeout extends CommandBase {
    private final IntakeBelt intakeBelt;
    private final double voltage;
    private final double timeout;
    private double startTime = 0.0;

    public SetBeltVoltageWithTimeout(double voltage, double timeout) {
        this.intakeBelt = IntakeBelt.getInstance();
        this.voltage = voltage;
        this.timeout = timeout;
        addRequirements(intakeBelt);
    }

    @Override
    public void initialize() {
        intakeBelt.setBeltVoltage(voltage);
        this.startTime = Timer.getFPGATimestamp();
    }

    //TODO this is a manual calculation of a timeout. Can you use .withTimeout() instead?

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= timeout;
    }

    @Override
    public void end(boolean interrupted) {
        intakeBelt.setBeltVoltage(0);
    }
}
