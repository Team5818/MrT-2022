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

package org.rivierarobotics.subsystems.intake;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.subsystems.climb.Piston;

public class IntakePiston extends SubsystemBase {
    private static IntakePiston INSTANCE;

    public static IntakePiston getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakePiston();
        }
        return INSTANCE;
    }

    private final Piston p1;
    private final Piston p2;
    private final Compressor compressor;

    public IntakePiston() {
        this.p1 = new Piston(0);
        this.p2 = new Piston(1);
        this.compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    }

    public void setIntakeState(boolean deploy) {
        p1.set(!deploy);
        p2.set(!deploy);
    }

    public boolean getIntakeState() {
        return p1.getState();
    }
}
