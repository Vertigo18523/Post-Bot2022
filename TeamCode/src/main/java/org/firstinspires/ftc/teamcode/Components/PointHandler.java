package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Base.Component;

import java.util.ArrayList;

public class PointHandler implements Component {

    public static class Point {
        double x;
        double y;
        double rot;
    }

    public ArrayList<Point> list = new ArrayList<>();
    private static final double PULSES_PER_REVOLUTION = 8192;
    private static final double WHEEL_DIAMETER_IN = 1.37795;
    public double ticksPerInch = PULSES_PER_REVOLUTION / (WHEEL_DIAMETER_IN * Math.PI);

    /**
     * Add a location
     * x, y are in inches,
     * rot is in degrees
     */
    public void addLoc(double x, double y, double rot) {
        // convert to Polar
        double r = Math.sqrt(Math.pow(x, 2.0) + Math.pow(y, 2.0)) * ticksPerInch;
        double theta = Math.atan2(y, x);
    }

    public void addLoc(Point point) {
        addLoc(point.x, point.y, point.rot);
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public String getTelemetry() {
        return null;
    }
}
