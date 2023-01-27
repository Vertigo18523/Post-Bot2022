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
public class NewArm implements Component {
    private final DcMotor rightArm;
    private final DcMotor leftArm;

    private final Telemetry telemetry;

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

    public double MotorPower;
    public int TotalTicks, StartingPosition;
    public boolean isTeleOp;
    public double proportional, integral = 0, derivative, pid, prevError = 0;
    public static double kP, kI, kD;
    ElapsedTime timer = new ElapsedTime();
    Telemetry telemetry1;


    public NewArm(
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
        this.telemetry = telemetry;
        this.isTeleOp = isTeleOp;
        this.telemetry1 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init() {
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        move(isTeleOp ? ZERO_POSITION : LOWER_BOUND);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        telemetry1.addData("Position", getCurrentPosition());
        telemetry1.addData("Target", getTargetPosition());
        if (isTeleOp) {
            if (isBusy()) {
//                setPower(MotorPower);
                proportional = TotalTicks - getCurrentPosition();
                integral += proportional * timer.seconds();
                derivative = (proportional - prevError) / timer.seconds();
                pid = (kP * proportional) + (kI * integral) + (kD * derivative);
                setPower(Math.min(pid, MotorPower));
                prevError = proportional;
                timer.reset();
//                setPower(((-4.0 * MotorPower) / Math.pow(TotalTicks, 2.0)) * Math.pow(TotalTicks / 2.0 - getCurrentPosition(), 2.0) + MotorPower);
            } else {
//                setPower(0);
                move(getTargetPosition());
            }
        } else {
            if (getCurrentPosition() != getTargetPosition()) move(getTargetPosition());
        }
        telemetry1.update();
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
        move(position, 1);
    }

    public void move(int position, double motorPower) {
        leftArm.setTargetPosition(position);
        rightArm.setTargetPosition(position);
//        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorPower = motorPower;
        TotalTicks = position;
        StartingPosition = getCurrentPosition();
        if (getCurrentPosition() + 10 > position && getCurrentPosition() > SIDE_STACK) { // if going down
            MotorPower *= 0.5;
        }
        if (!isTeleOp) {
            while (isBusy()) {
                setPower(MotorPower);
            }
//            setPower(0);
        }
    }

    public void setPower(double motorPower) {
        rightArm.setPower(motorPower);
        leftArm.setPower(motorPower);
    }

    public boolean isBusy() {
        return leftArm.isBusy() || rightArm.isBusy();
    }

    public int getCurrentPosition() {
        return Math.min(leftArm.getCurrentPosition(), rightArm.getCurrentPosition());
    }

    public int getTargetPosition() {
        return leftArm.getTargetPosition();
    }
}