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

public class PistonControl extends SubsystemBase {

    public static PistonControl getInstance() {
        return pistonControl;
    }

    private static PistonControl pistonControl;

    private static Solenoid low;
    private static Solenoid mid;
    private static Solenoid high;

    public PistonControl() {
        this.low = new Solenoid(PneumaticsModuleType.CTREPCM, Pistons.LOW.id);
        this.mid = new Solenoid(PneumaticsModuleType.CTREPCM, Pistons.MID.id);
        this.high = new Solenoid(PneumaticsModuleType.CTREPCM, Pistons.HIGH.id);
    }

    private Solenoid pistonFor(Pistons piston) {
        switch (piston) {
            case LOW:
                return low;
            case MID:
                return mid;
            case HIGH:
                return high;
            default:
                return null;
        }
    }


    public void set(Pistons piston, boolean isOpen) {
        pistonFor(piston).set(isOpen);
    }

    public boolean get(Pistons piston) {
        return pistonFor(piston).get();
    }

}
