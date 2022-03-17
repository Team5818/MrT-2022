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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbClaws extends SubsystemBase {
    private static ClimbClaws pistons;
    public static ClimbClaws getInstance() {
        if (pistons == null) {
            pistons = new ClimbClaws();
        }
        return pistons;
    }

    private ClimbClaws() {
    }

    public boolean isPistonSet(ClimbPositions climbModule) {
        return climbModule.piston.getState();
    }

    public void setPiston(ClimbPositions climbModule, boolean isEngaged) {
        climbModule.piston.set(isEngaged);
    }

    public void setAllPistons(boolean isOpen) {
        ClimbPositions.LOW.piston.set(isOpen);
        ClimbPositions.MID.piston.set(isOpen);
        ClimbPositions.HIGH.piston.set(isOpen);
    }

    public boolean isSwitchSet(ClimbPositions climbModule) {
        return !climbModule.input1.get() || !climbModule.input2.get();
    }
}
