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
    private Solenoid pistonSolenoid;
    private static Piston low;
    private static Piston mid;
    private static Piston high;
    public static Piston getInstanceLow() {
        if (low == null) {
            low = new Piston(PistonEnum.LOW, PneumaticsModuleType.CTREPCM);
        }
        return low;
    }
    public static Piston getInstanceMid() {
        if (mid == null) {
            mid = new Piston(PistonEnum.MID, PneumaticsModuleType.CTREPCM);
        }
        return mid;
    }
    public static Piston getInstanceHigh() {
        if (high == null) {
            high = new Piston(PistonEnum.HIGH, PneumaticsModuleType.CTREPCM);
        }
        return high;
    }

    public Piston(PistonEnum pistonEnum, PneumaticsModuleType moduleType) {
        pistonSolenoid = new Solenoid(moduleType, pistonEnum.id);

    }

    public void set(boolean isOpen) {
        pistonSolenoid.set(isOpen);
    }

    public boolean get() {
        return pistonSolenoid.get();
    }

    public enum PistonEnum {
        LOW(0),
        MID(1),
        HIGH(2);

        public final int id;

        PistonEnum(int id){
            this.id = id;
        }

    }
}


