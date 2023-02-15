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
    public double error, prevError = 0, time, prevTime = System.nanoTime() * 1e-9d;
    public static double kP = 0.01, kD, kG = 0.2;
    Telemetry telemetry;

    public Arm(
            String rightArmName,
            String leftArmName,
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
            double upperBound
    ) {
        rightArm = hardwareMap.get(DcMotor.class, rightArmName);
        leftArm = hardwareMap.get(DcMotor.class, leftArmName);

        rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);

        this.PULSES_PER_REVOLUTION = 145.1; // gobilda 5202 1150 rpm
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
        setPower(((kP * error) + (kD * -(error - prevError) / (time - prevTime)) + (targetPosition > 0 ? kG : 0.0)) * ((error < 0 && getCurrentPosition() > SIDE_STACK) ? (isTeleOp ? 0.3 : 1) : 1));
        prevError = error;
        prevTime = time;

        // https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops

        telemetry.addData("Position", getCurrentPosition());
        telemetry.addData("Target", getTargetPosition());
        telemetry.addData("Error", error);
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

    public void move(int position) {
        targetPosition = position;
        if (!isTeleOp) {
            while (isBusy()) {
                update();
            }
        }
    }

    public void setPower(double motorPower) {
        rightArm.setPower(motorPower);
        leftArm.setPower(motorPower);
    }

    public boolean isBusy() {
        return Math.abs(error) > 10;
    }

    public int getCurrentPosition() {
        return Math.min(leftArm.getCurrentPosition(), rightArm.getCurrentPosition());
    }

    public int getTargetPosition() {
        return leftArm.getTargetPosition();
    }
}