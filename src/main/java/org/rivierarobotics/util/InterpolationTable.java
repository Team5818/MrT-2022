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

package org.rivierarobotics.util;

import java.util.TreeMap;

public class InterpolationTable {
    private final TreeMap<Double, Double> interpolationTable;

    public InterpolationTable() {
        this.interpolationTable = new TreeMap<>();
    }

    public void addValue(double key, double value) {
        interpolationTable.put(key, value);
    }

    public double interpolateBetweenPoints(double x1, double y1, double x2, double y2, double interpolationVal) {
        var slope = (y2 - y1) / (x2 - x1);
        var intercept = y1 - slope * x1;
        return slope * interpolationVal + intercept;
    }

    public double getValue(double key) {
        if (interpolationTable.size() <= 1) {
            return 0.0;
        }
        if (interpolationTable.containsKey(key)) {
            return interpolationTable.get(key);
        }
        if (interpolationTable.floorKey(key) == null) {
            var firstValue = interpolationTable.firstEntry();
            var secondValue = interpolationTable.higherEntry(firstValue.getKey());
            return interpolateBetweenPoints(firstValue.getKey(), firstValue.getValue(), secondValue.getKey(), secondValue.getValue(), key);
        }
        if (interpolationTable.higherKey(key) == null) {
            var firstValue = interpolationTable.lastEntry();
            var secondValue = interpolationTable.lowerEntry(firstValue.getKey());
            return interpolateBetweenPoints(firstValue.getKey(), firstValue.getValue(), secondValue.getKey(), secondValue.getValue(), key);
        }

        var firstValue = interpolationTable.higherEntry(key);
        var secondValue = interpolationTable.lowerEntry(key);
        return interpolateBetweenPoints(firstValue.getKey(), firstValue.getValue(), secondValue.getKey(), secondValue.getValue(), key);
    }
}
