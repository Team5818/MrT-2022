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

package org.rivierarobotics.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbClaws extends SubsystemBase {
    private static ClimbClaws INSTANCE;

    public static ClimbClaws getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ClimbClaws();
        }
        return INSTANCE;
    }

    private ClimbClaws() {
    }

    public boolean isPistonSet(ClimbPositions climbModule) {
        return climbModule.pistonClosed.getState();
    }

    public void setPiston(ClimbPositions climbModule, boolean isEngaged) {
        climbModule.pistonClosed.set(!isEngaged);
        climbModule.pistonOpen.set(isEngaged);
    }

    public boolean isSwitchSet(ClimbPositions climbModule) {
        return !climbModule.input1.get() || !climbModule.input2.get();
    }
}
