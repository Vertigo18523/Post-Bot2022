package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Base.Component;
import org.firstinspires.ftc.teamcode.RMath.Point;
import org.firstinspires.ftc.teamcode.RMath.Util;
import org.firstinspires.ftc.teamcode.RMath.Vector;
import org.firstinspires.ftc.teamcode.Utils.PIDController;

import java.io.File;

public class PIDMecanum implements Component {

    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;

    public double flPower;
    public double frPower;
    public double blPower;
    public double brPower;

    public Odometry odo;
    public PIDController strafePID;
    public PIDController forwardPID;
    public PIDController rotPID;

    public double heldRotation = 0;
    public Point heldPosition = new Point(0, 0);

    public double xPower = 0;
    public double yPower = 0;
    public double turnPower = 0;

    public double positionTolerance = 0.1;
    public double rotationTolerance = 0.2;

    public double forwardStaticFriction = 0;
    public double forwardKineticFriction = 0;
    public double strafeStaticFriction = 0;
    public double strafeKineticFriction = 0;
    public double rotationalStaticFriction = 0;
    public double rotationalKineticFriction = 0;

    private boolean stoppedMoving = false;
    private boolean stoppedTurning = false;

    public double maxSpeed = 1;

    private HardwareMap.DeviceMapping<VoltageSensor> voltageSensor;

    public PIDMecanum(HardwareMap hardwareMap, String fl, String fr, String bl, String br, Odometry odo) {
        this.fl = hardwareMap.dcMotor.get(fl);
        this.fr = hardwareMap.dcMotor.get(fr);
        this.bl = hardwareMap.dcMotor.get(bl);
        this.br = hardwareMap.dcMotor.get(br);

        this.fl.setDirection(DcMotorSimple.Direction.FORWARD);
        this.fr.setDirection(DcMotorSimple.Direction.REVERSE);
        this.bl.setDirection(DcMotorSimple.Direction.FORWARD);
        this.br.setDirection(DcMotorSimple.Direction.REVERSE);

//        this.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.odo = odo;

        this.voltageSensor = hardwareMap.voltageSensor;

        readFiles();
    }

    public void drive(double x, double y, double rot) {

        if (rot == 0) {
            if(!stoppedTurning && Math.abs(odo.getAngularVelocity()) < 10) {
                stoppedTurning = true;
                heldRotation = odo.getRotation();
            }

            if(stoppedTurning){
                holdRotation();
            }else{
                turnPower = Util.cap(rot, -1, 1);
            }

        }else{
            turnPower = Util.cap(rot, -1, 1);
            stoppedTurning = false;
        }

        if (x == 0 && y == 0) {
            if(!stoppedMoving && odo.getSpeed() < 1) {
                stoppedMoving = true;
                heldPosition = odo.getPosition();
            }

            if(stoppedMoving){
                holdPosition();
            }else{
                xPower = x;
                yPower = y;
            }
        } else {
            stoppedMoving = false;
            xPower = x;
            yPower = y;
        }
    }

    public void moveToPosition(Point p) {
        Vector error = new Vector(p).subtract(new Vector(odo.getPosition()));
        error.setTheta(error.getTheta() - Math.toRadians(odo.getRotation()));

        if (error.getMagnitude() < positionTolerance) {
            xPower = 0;
            yPower = 0;
            heldPosition = odo.getPosition();
        } else {
            xPower = strafePID.getPower(error.getX());
            yPower = forwardPID.getPower(error.getY());
        }
    }

    public void moveToPosition(double x, double y) {
        moveToPosition(new Point(x, y));
    }

    public void turnToRotation(double rotation) {
        double error;
        double dist = Util.loop(rotation, 0, 360) - odo.getLoopedRotation();

        if (Math.abs(dist) < 180)
            error = dist;
        else {
            if (dist > 0) error = dist - 360;
            else error = dist + 360;
        }

        if (Math.abs(error) < rotationTolerance) {
            turnPower = 0;
            heldRotation = odo.getRotation();
        } else {
            turnPower = -Util.cap(rotPID.getPower(error), -1, 1);
        }


    }

    public void holdPosition() {
        moveToPosition(heldPosition);
    }

    public void holdRotation() {
        turnToRotation(heldRotation);
    }

    public void polarDrive(double speed, double direction, double rotVel) {
        final double x = Math.cos(direction) * speed;
        final double y = Math.sin(direction) * speed;

        drive(x, y, rotVel);
    }

    public void drive(Gamepad gamepad) {
        final double x = gamepad.left_stick_x;
        final double y = -gamepad.left_stick_y;
        final double rot = gamepad.right_stick_x;

        drive(x, y, rot);
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

        if (xPower != 0) {
            int sign = Util.sign(xPower);
//                double min = odo.getSpeed() < 0.2? strafeStaticFriction : strafeKineticFriction;
            double min = odo.getSpeed() < 0.2 ? strafeStaticFriction : 0;
            double magnitude = Math.max(Math.abs(xPower), min);
            xPower = magnitude * sign;
        }

        if (yPower != 0) {
            int sign = Util.sign(yPower);
//                double min = odo.getSpeed() < 0.2? forwardStaticFriction : forwardKineticFriction;
            double min = odo.getSpeed() < 0.2 ? forwardStaticFriction : 0;
            double magnitude = Math.max(Math.abs(yPower), min);
            yPower = magnitude * sign;
        }

        if(turnPower != 0){
            int sign = Util.sign(turnPower);
            double min = Math.abs(odo.getAngularVelocity()) < 5? rotationalStaticFriction : 0;
            double magnitude = Math.max(Math.abs(turnPower), min);
            turnPower = magnitude * sign;
        }

        turnPower = Util.cap(turnPower, -1, 1);
        final double direction = Util.angle(xPower, yPower) - Math.PI / 4.0;
        final double speed = Math.min(Util.dist(xPower, yPower), maxSpeed);

        final double voltage = getBatteryVoltage();

        final double flRaw = (speed * Math.cos(direction) + turnPower) * (13 / voltage);
        final double frRaw = (speed * Math.sin(direction) - turnPower) * (13 / voltage);
        final double blRaw = (speed * Math.sin(direction) + turnPower) * (13 / voltage);
        final double brRaw = (speed * Math.cos(direction) - turnPower) * (13 / voltage);

        double maxWheelPower = Math.abs(frRaw);
        if (Math.abs(flRaw) > maxWheelPower) maxWheelPower = Math.abs(flRaw);
        if (Math.abs(blRaw) > maxWheelPower) maxWheelPower = Math.abs(blRaw);
        if (Math.abs(brRaw) > maxWheelPower) maxWheelPower = Math.abs(brRaw);
        if (Math.abs(frRaw) > maxWheelPower) maxWheelPower = Math.abs(frRaw);

        maxWheelPower = Math.min(maxWheelPower, speed + Math.abs(turnPower));

        final double multiplier = (speed + Math.abs(turnPower)) / maxWheelPower;

        flPower = flRaw * multiplier;
        frPower = frRaw * multiplier;
        blPower = blRaw * multiplier;
        brPower = brRaw * multiplier;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    @Override
    public String getTelemetry() {
        return null;
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }


    private void readFiles() {

        try {
            File file = AppUtil.getInstance().getSettingsFile("strafePID.txt");
            String[] lines = ReadWriteFile.readFile(file).split("\n");
            double kP = Double.parseDouble(lines[0]);
            double kI = Double.parseDouble(lines[1]);
            double kD = Double.parseDouble(lines[2]);

            strafePID = new PIDController(kP, kI, kD);
        } catch (NumberFormatException | IndexOutOfBoundsException e) {
            throw new IllegalArgumentException("Forward PID calibration file invalid, try recalibrating strafe PID");
        }

        try {
            File file = AppUtil.getInstance().getSettingsFile("forwardPID.txt");
            String[] lines = ReadWriteFile.readFile(file).split("\n");
            double kP = Double.parseDouble(lines[0]);
            double kI = Double.parseDouble(lines[1]);
            double kD = Double.parseDouble(lines[2]);

            forwardPID = new PIDController(kP, kI, kD);
        } catch (NumberFormatException | IndexOutOfBoundsException e) {
            throw new IllegalArgumentException("Forward PID calibration file invalid, try recalibrating forward PID");
        }

        try {
            File file = AppUtil.getInstance().getSettingsFile("rotPID.txt");
            String[] lines = ReadWriteFile.readFile(file).split("\n");
            double kP = Double.parseDouble(lines[0]);
            double kI = Double.parseDouble(lines[1]);
            double kD = Double.parseDouble(lines[2]);

            rotPID = new PIDController(kP, kI, kD);
        } catch (NumberFormatException | IndexOutOfBoundsException e) {
            throw new IllegalArgumentException("Forward PID calibration file invalid, try recalibrating rotational PID");
        }

        try {
            File file = AppUtil.getInstance().getSettingsFile("friction.txt");
            String[] lines = ReadWriteFile.readFile(file).split("\n");
            forwardStaticFriction = Double.parseDouble(lines[0]);
            forwardKineticFriction = Double.parseDouble(lines[1]);
            strafeStaticFriction = Double.parseDouble(lines[2]);
            strafeKineticFriction = Double.parseDouble(lines[3]);
            rotationalStaticFriction = Double.parseDouble(lines[4]);
            rotationalKineticFriction = Double.parseDouble(lines[5]);

        } catch (NumberFormatException | IndexOutOfBoundsException e) {
            throw new IllegalArgumentException("Friction calibration file invalid, try recalibrating friction values");
        }


    }
}