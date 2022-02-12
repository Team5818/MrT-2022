package org.rivierarobotics.util;

import java.util.TreeMap;

public class InterpolationTable {
    private TreeMap<Double, Double> interpolationTable;

    public InterpolationTable() {
        interpolationTable = new TreeMap<Double, Double>();
    }

    public void addValue(double key, double value){
        interpolationTable.put(key, value);
    }

    public double getValue(double key){
        if (interpolationTable.containsKey(key)) {
            return interpolationTable.get(key);
        } else if (key < interpolationTable.firstKey()) {
            double lowestValue = interpolationTable.get(interpolationTable.firstKey());
            double higherValue = interpolationTable.get(interpolationTable.higherKey(interpolationTable.firstKey()));
            return lowestValue - ((higherValue - lowestValue) / (interpolationTable.higherKey(interpolationTable.firstKey()) - interpolationTable.firstKey())) * (interpolationTable.firstKey() - key)
        } else if (key > interpolationTable.lastKey()){
            double highestValue = interpolationTable.get(interpolationTable.lastKey());
            double lowerKey = interpolationTable.get(interpolationTable.lowerKey(interpolationTable.lastKey()));
            return highestValue + ((highestValue - lowerKey) / (interpolationTable.lastKey() - interpolationTable.lowerKey(interpolationTable.lastKey())) * (key - interpolationTable.lastKey()));
        } else {
            double lowerValue = interpolationTable.get(interpolationTable.lowerKey(key));
            double higherValue = interpolationTable.get(interpolationTable.higherKey(key));
            return lowerValue + ((higherValue - lowerValue) / (interpolationTable.lowerKey(key) - interpolationTable.higherKey(key))) * (key - interpolationTable.lowerKey(key));
        }
    }


}
