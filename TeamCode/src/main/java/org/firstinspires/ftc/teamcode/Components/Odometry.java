package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Base.Component;
import org.firstinspires.ftc.teamcode.RMath.Point;
import org.firstinspires.ftc.teamcode.RMath.PolarPoint;
import org.firstinspires.ftc.teamcode.RMath.Util;
import org.firstinspires.ftc.teamcode.RMath.Vector;
import org.firstinspires.ftc.teamcode.Utils.Differentiator;
import org.firstinspires.ftc.teamcode.Utils.Logger;

import java.io.File;

public class Odometry implements Component {

    /**
     * The heading of the robot in degrees (not limited from 0 to 360)
     */
    private double rotation;
    private double x, y;

    private double rotationOffset = 0;
    private double xOffset = 0, yOffset = 0;

    private Differentiator rotationDiff = new Differentiator(2, 0, true);
    private Differentiator xDiff = new Differentiator(2, 0, true);
    private Differentiator yDiff = new Differentiator(2, 0, true);

    public DcMotorEx left;
    public DcMotorEx strafe;
    public DcMotorEx right;

    public EncoderDirection leftDir = EncoderDirection.REVERSE;
    public EncoderDirection strafeDir = EncoderDirection.REVERSE;
    public EncoderDirection rightDir = EncoderDirection.REVERSE;

    public Logger logger = new Logger();

    public static class LOG_LEVEL {
        public static final int ERROR = 1;
        public static final int IMPORTANT = 2;
        public static final int INFO = 3;
    }

    private double ticksPerDegree;
    private double strafeTicksPerDegree;


    public double omniWheelDiameter = 1.37795; // in
    public double ticksPerRevolution = 8192;
    public double ticksPerInch = ticksPerRevolution / (omniWheelDiameter * Math.PI);

    public Differentiator leftPosDiff = new Differentiator(1, 0, false);
    public Differentiator strafePosDiff = new Differentiator(1, 0, false);
    public Differentiator rightPosDiff = new Differentiator(1, 0, false);

    private double forwardDist = 0;
    private double strafeDist = 0;

    /***
     *
     * @param map HardwareMap
     * @param left
     * @param strafe
     * @param right
     */
    public Odometry(HardwareMap map, String left, String strafe, String right) {

        this.left = map.get(DcMotorEx.class, left);
        this.strafe = map.get(DcMotorEx.class, strafe);
        this.right = map.get(DcMotorEx.class, right);

//        for (LynxModule hub : map.getAll(LynxModule.class)) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
        readFiles();
    }

    @Override
    public void init() {
        resetEncoders();
    }

    public void resetEncoders(){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

        logger.clear();

//        Fetch translations with each update
        leftPosDiff.update(left.getCurrentPosition() * (leftDir == EncoderDirection.REVERSE? -1 : 1) * (left.getDirection() == DcMotorSimple.Direction.REVERSE? -1 : 1));
        strafePosDiff.update(strafe.getCurrentPosition() * (strafeDir == EncoderDirection.REVERSE? -1 : 1) * (strafe.getDirection() == DcMotorSimple.Direction.REVERSE? -1 : 1));
        rightPosDiff.update(right.getCurrentPosition() * (rightDir == EncoderDirection.REVERSE? -1 : 1) * (right.getDirection() == DcMotorSimple.Direction.REVERSE? -1 : 1));

        final double rightDelta = rightPosDiff.getDerivative(1);
        final double leftDelta = leftPosDiff.getDerivative(1);

        final double deltaRot = (rightDelta - leftDelta) / (2 * ticksPerDegree);

        final double rightDeltaForward = rightDelta / ticksPerInch;
        final double leftDeltaForward = leftDelta / ticksPerInch;


        rotation = (rightPosDiff.getValue() - leftPosDiff.getValue()) / (2 * ticksPerDegree);
        rotationDiff.update(rotation);

        final double forwardDelta = (rightDeltaForward + leftDeltaForward) / 2;
        forwardDist += forwardDelta;
        final double strafeDelta = (strafePosDiff.getDerivative(1) - deltaRot * strafeTicksPerDegree) / ticksPerInch;
        strafeDist += strafeDelta;

//        logger.log(LOG_LEVEL.INFO, "Delta right: %g", rightDeltaForward);
//        logger.log(LOG_LEVEL.INFO, "Delta left: %g", leftDeltaForward);
//        logger.log(LOG_LEVEL.INFO, "Delta forward: %g", forwardDelta);
//        logger.log(LOG_LEVEL.INFO, "Delta strafe: %g", strafeDelta);
//        logger.log(LOG_LEVEL.INFO, "Delta heading: %g", deltaRot);


        if (deltaRot != 0) {
//           This is a method to generate an arc to better replicate the movement of the robot
//           It plugs our initial heading, final heading, delta strafe, and delta forward values
//           into a polar equation, which gives us a polar point output which we then convert into a
//           rectangular output
            final double initialHeading = Math.toRadians(rotation - deltaRot);
            final double finalHeading = Math.toRadians(rotation);
            final double turnRadius = forwardDelta / Math.toRadians(deltaRot);
            PolarPoint destination = new PolarPoint(turnRadius + strafeDelta, finalHeading);
            Point start = new Point(new PolarPoint(turnRadius, initialHeading));
            Point relation = start.getRelation(new Point(destination));
            x += relation.x;
            y += relation.y;

        } else {

            PolarPoint destination = new PolarPoint(new Point(strafeDelta, forwardDelta));
            destination.theta += rotation;
            x += new Point(destination).x;
            y += new Point(destination).y;
        }

        xDiff.update(x);
        yDiff.update(y);

        logger.log(LOG_LEVEL.IMPORTANT, "Position: (%g, %g)", xDiff.getValue(), yDiff.getValue());
        logger.log(LOG_LEVEL.IMPORTANT, "Heading: %g", rotationDiff.getValue());
        logger.log(LOG_LEVEL.INFO, "left: %g", leftPosDiff.getValue());
        logger.log(LOG_LEVEL.INFO, "strafe: %g", strafePosDiff.getValue());
        logger.log(LOG_LEVEL.INFO, "right: %g", rightPosDiff.getValue());
        logger.log(LOG_LEVEL.INFO, "Velocity: (%g, %g)", xDiff.getDerivative(1), yDiff.getDerivative(1));
        logger.log(LOG_LEVEL.INFO, "Acceleration: (%g, %g)", xDiff.getDerivative(2), yDiff.getDerivative(2));

    }


    @Override
    public String getTelemetry() {
        return logger.getData();
    }


    public void set(double x, double y, double r) {
        xOffset = x - this.x;
        yOffset = y - this.y;
        rotationOffset = r - this.rotation;
    }

    public void set(Point p, double r) {
        set(p.x, p.y, r);
    }

    //    Get the rotation in degrees in the range of 0 to 360
    public double getLoopedRotation() {
        return (getRotation() % 360 + 360) % 360;
    }

    private void readFiles() {
        AppUtil instance = AppUtil.getInstance();
        File ticksPerDegreeFile = instance.getSettingsFile("odoTicksPerDegree.txt");
        File strafeTicksPerDegreeFile = instance.getSettingsFile("odoStrafeTicksPerDegree.txt");

        ticksPerDegree = Double.parseDouble(ReadWriteFile.readFile(ticksPerDegreeFile).trim());
        strafeTicksPerDegree = Double.parseDouble(ReadWriteFile.readFile(strafeTicksPerDegreeFile).trim());
    }

    public enum EncoderDirection {
        FORWARD,
        REVERSE
    }

    public double getSpeed(){
        return Util.dist(xDiff.getDerivative(1), yDiff.getDerivative(1));
    }

    public Vector getVelocity(){
        return new Vector(new Point(xDiff.getDerivative(1), yDiff.getDerivative(1)));
    }

    public Vector getAcceleration(){
        return new Vector(new Point(xDiff.getDerivative(2), yDiff.getDerivative(2)));
    }

    /***
     *
     * @return The distance that the robot has traveled while moving forward
     */
    public double getForwardDist(){
        return forwardDist;
    }

    /***
     *
     * @return The distance that the robot has traveled while strafing
     */
    public double getStrafeDist(){
        return strafeDist;
    }

    public Vector getLocalVelocity(){
        Vector local = getVelocity().clone();
        local.setTheta(local.getTheta() - Math.toRadians(rotation));
        return local;
    }

    public double getX() {
        return x + xOffset;
    }

    public double getY() {
        return y + yOffset;
    }

    public double getRotation(){
        return rotation + rotationOffset;
    }

    public double getAngularVelocity(){
        return rotationDiff.getDerivative(1);
    }

    public Point getPosition() {
        return new Point(getX(), getY());
    }


}