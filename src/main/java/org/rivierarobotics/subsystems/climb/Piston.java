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

package org.rivierarobotics.subsystems.climb;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Piston extends SubsystemBase {

    public static Piston getInstance(Pistons piston) {
        switch (piston) {
            case LOW:
                if (low == null) {
                    low = new Piston(0);
                }
                return low;
            case MID:
                if (mid == null) {
                    mid = new Piston(1);
                }
                return mid;
            case HIGH:
                if (high == null) {
                    high = new Piston(2);
                }
                return high;
            default:
                return null;
        }
    }

    private static Piston low;
    private static Piston mid;
    private static Piston high;

    public enum Pistons {
        LOW,
        MID,
        HIGH
    }

    private Solenoid pistonSolenoid;

    public Piston(int id, PneumaticsModuleType moduleType) {
        this.pistonSolenoid = new Solenoid(moduleType, id);
    }

    public Piston(int id) {
        this.pistonSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, id);
    }

    public void set(boolean isOpen) {
        pistonSolenoid.set(isOpen);
    }

    public boolean get() {
        return pistonSolenoid.get();
    }

}


