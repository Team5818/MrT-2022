package org.rivierarobotics.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.TreeMap;

public class InterpolationTable {
    private TreeMap<Double, Double> interpolationTable;

    public InterpolationTable() {
        interpolationTable = new TreeMap<Double, Double>();
    }

    public void addValue(double key, double value) {
        interpolationTable.put(key, value);
    }

    public double interpolateBetweenPoints(double x1, double y1, double x2, double y2, double interpolationVal) {
        var slope = (y2 - y1) / (x2 - x1);
        var intercept = y1 - slope * x1;
        return slope * interpolationVal + intercept;
    }

    public double getValue(double key){
        if(interpolationTable.size() <= 1) return 0.0;
        if(interpolationTable.containsKey(key)) return interpolationTable.get(key);
        if(interpolationTable.floorKey(key) == null) {
            var firstValue = interpolationTable.firstEntry();
            var secondValue = interpolationTable.higherEntry(firstValue.getKey());
            return interpolateBetweenPoints(firstValue.getKey(), firstValue.getValue(), secondValue.getKey(), secondValue.getValue(), key);
        }
        if(interpolationTable.higherKey(key) == null) {
            var firstValue = interpolationTable.lastEntry();
            var secondValue = interpolationTable.lowerEntry(firstValue.getKey());
            return interpolateBetweenPoints(firstValue.getKey(), firstValue.getValue(), secondValue.getKey(), secondValue.getValue(), key);
        }

        var firstValue = interpolationTable.higherEntry(key);
        var secondValue = interpolationTable.lowerEntry(key);
        return interpolateBetweenPoints(firstValue.getKey(), firstValue.getValue(), secondValue.getKey(), secondValue.getValue(), key);
    }


}
