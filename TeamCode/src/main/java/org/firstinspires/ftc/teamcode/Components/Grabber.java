package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

@Config
public class Grabber implements Component {
    private final Servo grabber;

    private final Telemetry telemetry;
    private final LinearOpMode opMode;

    public static double OPEN;
    public static double CLOSED;

    public boolean isGrabbing;

    public Grabber(LinearOpMode opMode, String deviceName, HardwareMap hardwareMap, Telemetry telemetry, double open, double closed) {
        grabber = hardwareMap.get(Servo.class, deviceName);
        OPEN = open;
        CLOSED = closed;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.opMode = opMode;
    }

    @Override
    public void init() {
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        updatePos();
    }

    @Override
    public String getTelemetry() {
        telemetry.addData("GrabberCurrentPosition", grabber.getPosition());
        telemetry.addData("GrabberTargetPosition", isGrabbing ? CLOSED : OPEN);
        telemetry.addData("isGrabbing", isGrabbing);
        return null;
    }

    public void toggle() {
        this.isGrabbing = !this.isGrabbing;
    }

    public void open() {
        this.isGrabbing = false;
        updatePos();
    }

    public void close() {
        this.isGrabbing = true;
        updatePos();
    }

    public void updatePos() {
        grabber.setPosition(this.isGrabbing ? CLOSED : OPEN);
    }
}
