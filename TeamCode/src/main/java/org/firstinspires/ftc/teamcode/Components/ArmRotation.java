package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

@Config
public class ArmRotation implements Component {
    private final DcMotor rotation;

    public double PULSES_PER_REVOLUTION;
    public int INIT;
    public int FORWARD;
    public int BACKWARD;
    public static int targetPosition = 0;
    public boolean isTeleOp;
    public double error, prevError = 0, time, prevTime = System.nanoTime() * 1e-9d, power;
    public static double kP = 0.04, kD = 0, kG = 0;
    Telemetry telemetry;

    public ArmRotation(
        String rotationName,
        HardwareMap hardwareMap,
        Telemetry telemetry,
        boolean isTeleOp,
        double init,
        double forward,
        double backward
    ) {
        rotation = hardwareMap.get(DcMotor.class, rotationName);

        rotation.setDirection(DcMotorSimple.Direction.FORWARD);

        this.PULSES_PER_REVOLUTION = 751.8;
        this.INIT = (int) (init * PULSES_PER_REVOLUTION);
        this.FORWARD = (int) (forward * PULSES_PER_REVOLUTION);
        this.BACKWARD = (int) (backward * PULSES_PER_REVOLUTION);
        this.isTeleOp = isTeleOp;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init() {
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        error = targetPosition - getCurrentPosition();
        time = System.nanoTime() * 1e-9d;
        power = (kP * error) + (kD * -(error - prevError) / (time - prevTime)) + (kG * Math.cos(Math.toRadians(targetPosition)));
        setPower(power);
        prevError = error;
        prevTime = time;

    }

    @Override
    public String getTelemetry() {
        telemetry.addData("RotationPosition", getCurrentPosition());
        telemetry.addData("RotationTarget", targetPosition);
        telemetry.addData("RotationError", error);
        telemetry.addData("RotationPower", power);
        return null;
    }

    public void toInit() {
        move(INIT);
    }

    public void toForward() {
        move(FORWARD);
    }

    public void toBackward() {
        move(BACKWARD);
    }

    public void toggle() {
        if (getCurrentPosition() + error == BACKWARD) {
            toForward();
        } else if (getCurrentPosition() + error == FORWARD) {
            toBackward();
        } else { // init position
            toBackward();
        }
    }

    public void move(int position) {
        targetPosition = position;
        if (!isTeleOp) {
            while (isBusy()) {
                update();
            }
        }
    }

    public void setPower(double motorPower) {
        if (motorPower > 1) motorPower = 1;
        rotation.setPower(motorPower);
    }

    public boolean isBusy() {
        return Math.abs(error) > 10;
    }

    public int getCurrentPosition() {
        return rotation.getCurrentPosition();
    }
}
