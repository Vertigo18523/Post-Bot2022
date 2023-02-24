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
public class Slides implements Component {
    private final DcMotor rightArm;
    private final DcMotor leftArm;
    public ArmRotation rotation;
    public double PULSES_PER_REVOLUTION;
    public int LOWER_BOUND;
    public int ZERO_POSITION;
    public int GROUND_JUNCTION;
    public int PICKUP;
    public int SIDE_STACK;
    public int LOW_JUNCTION;
    public int MEDIUM_JUNCTION;
    public int HIGH_JUNCTION;
    public int UPPER_BOUND;
    public static int targetPosition = 0;
    public boolean isTeleOp;
    public double error, prevError = 0, time, prevTime = System.nanoTime() * 1e-9d, power;
    public static double kP = 0.015, kD = 0, kG = 0.2;
    Telemetry telemetry;

    public Slides(
            String rightArmName,
            String leftArmName,
            HardwareMap hardwareMap,
            Telemetry telemetry,
            boolean isTeleOp,
            ArmRotation rotation,
            double lowerBound,
            double zeroPosition,
            double groundJunction,
            double pickup,
            double sideStack,
            double lowJunction,
            double mediumJunction,
            double highJunction,
            double upperBound
    ) {
        rightArm = hardwareMap.get(DcMotor.class, rightArmName);
        leftArm = hardwareMap.get(DcMotor.class, leftArmName);

        rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);

        this.PULSES_PER_REVOLUTION = 384.5;
        this.LOWER_BOUND = (int) (lowerBound * PULSES_PER_REVOLUTION);
        this.ZERO_POSITION = (int) (zeroPosition * PULSES_PER_REVOLUTION);
        this.GROUND_JUNCTION = (int) (groundJunction * PULSES_PER_REVOLUTION);
        this.LOW_JUNCTION = (int) (lowJunction * PULSES_PER_REVOLUTION);
        this.PICKUP = (int) (pickup * PULSES_PER_REVOLUTION);
        this.SIDE_STACK = (int) (sideStack * PULSES_PER_REVOLUTION);
        this.MEDIUM_JUNCTION = (int) (mediumJunction * PULSES_PER_REVOLUTION);
        this.HIGH_JUNCTION = (int) (highJunction * PULSES_PER_REVOLUTION);
        this.UPPER_BOUND = (int) (upperBound * PULSES_PER_REVOLUTION);

        this.isTeleOp = isTeleOp;
        this.rotation = rotation;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init() {
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move(isTeleOp ? ZERO_POSITION : LOWER_BOUND);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        error = targetPosition - getCurrentPosition();
        time = System.nanoTime() * 1e-9d;
        power = ((kP * error) + (kD * -(error - prevError) / (time - prevTime)) + (targetPosition > 0 ? kG : 0.0));// * ((error < 0 && getCurrentPosition() > SIDE_STACK) ? (isTeleOp ? 0.3 : 1) : 1));
        setPower(power);
        prevError = error;
        prevTime = time;
    }

    @Override
    public String getTelemetry() {
        telemetry.addData("SlidePosition", getCurrentPosition());
        telemetry.addData("SlideTarget", targetPosition);
        telemetry.addData("SlideError", error);
        telemetry.addData("SlidePower", power);
        telemetry.addData("Left", leftArm.getCurrentPosition());
        telemetry.addData("Right", rightArm.getCurrentPosition());
        return null;
    }

    public void toZero() {
        move(ZERO_POSITION);
        rotation.toForward();
    }

    public void toGround() {
        move(GROUND_JUNCTION);
        rotation.toForward();
    }

    public void toPickup() {
        move(PICKUP);
        rotation.toForward();
    }

    public void toSideStack() {
        move(SIDE_STACK);
        rotation.toForward();
    }

    public void toLow() {
        move(LOW_JUNCTION);
        rotation.toBackward();
    }

    public void toMedium() {
        move(MEDIUM_JUNCTION);
        rotation.toBackward();
    }

    public void toHigh() {
        move(HIGH_JUNCTION);
        rotation.toBackward();
    }

    public void move(int position) {
        targetPosition = position;
//        if (!isTeleOp) {
//            while (isBusy()) {
//                update();
//            }
//        }
    }

    public void setPower(double motorPower) {
        if (motorPower > 1) motorPower = 1;
        rightArm.setPower(motorPower);
        leftArm.setPower(motorPower);
    }

    public boolean isBusy() {
        return Math.abs(error) > 10;
    }

    public int getCurrentPosition() {
        return Math.min(leftArm.getCurrentPosition(), rightArm.getCurrentPosition());
    }
}