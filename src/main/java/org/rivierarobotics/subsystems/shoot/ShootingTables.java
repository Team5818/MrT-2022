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

package org.rivierarobotics.subsystems.shoot;

import org.rivierarobotics.util.InterpolationTable;

public class ShootingTables {
    public static InterpolationTable getFloppaAngleTable() {
        var angleTable = new InterpolationTable();

        angleTable.addValue(2.684, 12.6);
        angleTable.addValue(1.2, 9.36);
        angleTable.addValue(4.62, 14.261);

        return angleTable;
    }

    public static InterpolationTable getFloppaSpeedTable() {
        var speedTable = new InterpolationTable();

        speedTable.addValue(2.684, 7000 * 1.2);
        speedTable.addValue(4.62, 9400 * 1.2);
        speedTable.addValue(1.2, 5700 * 1.2);
        speedTable.addValue(3.14, 8100 * 1.2);

        return speedTable;
    }

    private ShootingTables() {
    }
}
