package org.firstinspires.ftc.teamcode.Utils;

import java.util.ArrayList;
import java.util.List;

public class AverageFilter implements Filter {

    List<Double> previousValues = new ArrayList<>();
    public int numValues;

    public AverageFilter(int numValues){
        this.numValues = numValues;
    }

    @Override
    public double getFilteredValue(double value) {
        previousValues.add(value);
        while(previousValues.size() > numValues){
            previousValues.remove(0);
        }

        return computeAverage();
    }

    private double computeAverage(){
        double sum = 0;
        for(int i = 0; i < previousValues.size(); i++){
            sum += previousValues.get(i);
        }

        return sum / previousValues.size();
    }
}