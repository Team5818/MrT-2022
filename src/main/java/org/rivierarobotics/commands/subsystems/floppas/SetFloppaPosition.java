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

package org.rivierarobotics.commands.subsystems.floppas;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.shoot.Floppas;

public class SetFloppaPosition extends CommandBase {
    private final double flywheelRads;
    private final Floppas floppas;
    public SetFloppaPosition(double flywheelRads) {
        this.flywheelRads = flywheelRads;
        this.floppas = Floppas.getInstance();
        addRequirements(floppas);
    }

    public SetFloppaPosition(Floppas.ShooterLocations preset) {
        this(preset.floppaAngle);
    }

    @Override
    public void initialize() {
        this.floppas.setAngle(flywheelRads);
    }

    @Override
    public void execute() {
        this.floppas.floppaStateSpaceControl();
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(floppas.getAngle(), flywheelRads, 0.15);
    }
}
