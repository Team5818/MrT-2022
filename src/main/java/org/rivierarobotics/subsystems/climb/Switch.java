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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//TODO: REMOVE CLASS - Move getState to Climb. DigitalInputs/Switches are simple enough not to need their own class
public class Switch extends SubsystemBase {

    public static Switch getInstance(Buttons button) {
        switch (button) {
            case LOW:
                if (low == null) {
                    low = new Switch(0);
                }
                return low;
            case MID:
                if (mid == null) {
                    mid = new Switch(1);
                }
                return mid;
            case HIGH:
                if (high == null) {
                    high = new Switch(2);
                }
                return high;
            default:
                return null;
        }
    }

    private static Switch low;
    private static Switch mid;
    private static Switch high;

    //TODO: Move Enum into climb - rename ClimbSwitches
    public enum Buttons {
        LOW,
        MID,
        HIGH
    }

    //Verify IDs

    private int id;
    private DigitalInput digitalInput;

    public Switch(int id) {
        this.id = id;
        this.digitalInput = new DigitalInput(id);
    }

    public boolean getState() {
        return !digitalInput.get();
    }
}
