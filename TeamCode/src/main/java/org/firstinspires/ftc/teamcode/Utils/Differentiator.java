package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.RMath.Util;

import java.util.ArrayList;
import java.util.List;

public class Differentiator {

    private double[] derivatives;
    private double[] prevDerivatives;
    private double[] integrals;
    private Double[] integralSaturations;

    public boolean useTime;
    private List<Filter>[] filters;

    private long prevTime = System.nanoTime();

    public Differentiator(int derivatives, int integrals, boolean useTime) {

        this.useTime = useTime;

//        The derivatives list includes the actual value so +1 is needed for the length
        this.derivatives = new double[derivatives + 1];
        this.prevDerivatives = new double[derivatives + 1];
        this.integrals = new double[integrals];
        this.integralSaturations = new Double[integrals];
        this.filters = new List[derivatives + 1];
        for (int i = 0; i < derivatives + 1; i++) {
            filters[i] = new ArrayList<>();
        }

        for(int i = 0; i < integrals; i++){
            integralSaturations[i] = Double.POSITIVE_INFINITY;
        }

    }

    public void update(double value) {
        double deltaTime = 0;
        if (useTime) {
            long time = System.nanoTime();
//            Converting nanoseconds to seconds
            deltaTime = (time - prevTime) / 1e9d;
            prevTime = time;
        }

        prevDerivatives[0] = derivatives[0];
        derivatives[0] = value;

        for (int i = 1; i < derivatives.length; i++) {
            double unfiltered = (derivatives[i - 1] - prevDerivatives[i - 1]) / (useTime ? deltaTime : 1.);
            prevDerivatives[i] = derivatives[i];

            for(Filter f : filters[i]){
                unfiltered = f.getFilteredValue(unfiltered);
            }

            derivatives[i] = unfiltered;
        }

        if (integrals.length > 0){
            integrals[0] += value;
            integrals[0] = Util.absCap(integrals[0], 0, integralSaturations[0]);
            Util.absCap(integrals[0], 0, integralSaturations[0]);
        }
        for (int i = 1; i < integrals.length; i++) {
            integrals[i] += integrals[i - 1];
            integrals[i] = Util.absCap(integrals[i], 0, integralSaturations[i]);
        }
    }

    public void addFilter(int derivative, Filter f){
        filters[derivative].add(f);
    }

    public double getDerivative(int number) {
        return derivatives[number];
    }


    public double getIntegral(int number) {
        return integrals[number - 1];
    }

    public double getValue() {
        return derivatives[0];
    }

    public void addIntegralSaturation(double saturation, int integral){
        integralSaturations[integral - 1] = saturation;
    }

    public double getIntegralSaturation(int integral){
        return integralSaturations[integral - 1];
    }

}