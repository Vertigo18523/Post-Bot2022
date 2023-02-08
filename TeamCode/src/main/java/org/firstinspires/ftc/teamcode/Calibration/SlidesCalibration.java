package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class SlidesCalibration extends LinearOpMode {

    public static double kp = 0.0;
    public static double kd = 0.0;
    public static double kg = 0.2;
    public static int targetHeight = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor left = hardwareMap.dcMotor.get("leftArm");
        final DcMotor right = hardwareMap.dcMotor.get("arm");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        double lastError = 0.0;
        double lastTime = System.nanoTime() * 1e-9d;

        while(opModeIsActive()){
            final int height = Math.min(left.getCurrentPosition(), right.getCurrentPosition());
            final int error = targetHeight - height;
            final double time = System.nanoTime() * 1e-9d;
            final double deltaTime = time - lastTime;
            lastTime = time;

            final double deltaError = -(error - lastError) / deltaTime;

            final double power = kp * error + kd * deltaError + (targetHeight > 0? kg : 0.0);

            telemetry.addData("height", height);
            telemetry.addData("targetHeight", targetHeight);
            telemetry.addData("error", error);
            telemetry.update();

            left.setPower(power);
            right.setPower(power);
            lastError = error;
        }
    }
}
