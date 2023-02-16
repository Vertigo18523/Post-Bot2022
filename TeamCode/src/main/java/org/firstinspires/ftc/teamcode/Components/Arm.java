package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

@Config
public class Arm implements Component {
    private final DcMotor rightArm;
    private final DcMotor leftArm;
    private final DcMotor rotation;
    public double PULSES_PER_REVOLUTION;
    public double PULSES_PER_REVOLUTION_ROTATION;
    public int LOWER_BOUND;
    public int ZERO_POSITION;
    public int GROUND_JUNCTION;
    public int PICKUP;
    public int SIDE_STACK;
    public int LOW_JUNCTION;
    public int MEDIUM_JUNCTION;
    public int HIGH_JUNCTION;
    public int UPPER_BOUND;
    public int FORWARD;
    public int BACKWARD;
    public static int targetPosition = 0;
    public boolean isTeleOp;
    public double error, prevError = 0, time, prevTime = System.nanoTime() * 1e-9d, power;
    public static double kP, kD, kG;
    Telemetry telemetry;

    public Arm(
            String rightArmName,
            String leftArmName,
            String rotationName,
            HardwareMap hardwareMap,
            Telemetry telemetry,
            boolean isTeleOp,
            double lowerBound,
            double zeroPosition,
            double groundJunction,
            double pickup,
            double sideStack,
            double lowJunction,
            double mediumJunction,
            double highJunction,
            double upperBound,
            double forward,
            double backward
    ) {
        rightArm = hardwareMap.get(DcMotor.class, rightArmName);
        leftArm = hardwareMap.get(DcMotor.class, leftArmName);
        rotation = hardwareMap.get(DcMotor.class, rotationName);

        rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);

        this.PULSES_PER_REVOLUTION = 145.1; // gobilda 5202 1150 rpm
        this.PULSES_PER_REVOLUTION_ROTATION = 145.1; // gobilda 5202 1150 rpm
        this.LOWER_BOUND = (int) (lowerBound * PULSES_PER_REVOLUTION);
        this.ZERO_POSITION = (int) (zeroPosition * PULSES_PER_REVOLUTION);
        this.GROUND_JUNCTION = (int) (groundJunction * PULSES_PER_REVOLUTION);
        this.LOW_JUNCTION = (int) (lowJunction * PULSES_PER_REVOLUTION);
        this.PICKUP = (int) (pickup * PULSES_PER_REVOLUTION);
        this.SIDE_STACK = (int) (sideStack * PULSES_PER_REVOLUTION);
        this.MEDIUM_JUNCTION = (int) (mediumJunction * PULSES_PER_REVOLUTION);
        this.HIGH_JUNCTION = (int) (highJunction * PULSES_PER_REVOLUTION);
        this.UPPER_BOUND = (int) (upperBound * PULSES_PER_REVOLUTION);
        this.FORWARD = (int) (forward * PULSES_PER_REVOLUTION_ROTATION);
        this.BACKWARD = (int) (backward * PULSES_PER_REVOLUTION_ROTATION);
        this.isTeleOp = isTeleOp;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init() {
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        // https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops

        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotation.setPower(0.5);

        telemetry.addData("Position", getCurrentPosition());
        telemetry.addData("Target", targetPosition);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    @Override
    public String getTelemetry() {
        return "Left: " + leftArm.getCurrentPosition() + " Right: " + rightArm.getCurrentPosition();
    }

    public void toZero() {
        move(ZERO_POSITION);
    }

    public void toGround() {
        move(GROUND_JUNCTION);
    }

    public void toPickup() {
        move(PICKUP);
    }

    public void toSideStack() {
        move(SIDE_STACK);
    }

    public void toLow() {
        move(LOW_JUNCTION);
    }

    public void toMedium() {
        move(MEDIUM_JUNCTION);
    }

    public void toHigh() {
        move(HIGH_JUNCTION);
    }

    public void toForward() {
        moveRotation(FORWARD);
    }

    public void toBackward() {
        moveRotation(BACKWARD);
    }

    public void move(int position) {
        targetPosition = position;
        if (!isTeleOp) {
            while (isBusy()) {
                update();
            }
        }
    }

    public void moveRotation(int position) {
        rotation.setTargetPosition(position);
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