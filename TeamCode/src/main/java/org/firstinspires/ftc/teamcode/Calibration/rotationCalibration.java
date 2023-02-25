package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Config
@TeleOp
@Disabled
public class rotationCalibration extends LinearOpMode {
    DcMotor rotation;
    public static int targetPosition = 0;
    public static double power = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        rotation = hardwareMap.get(DcMotor.class, "rotation");
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            rotation.setTargetPosition(targetPosition);
            rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotation.setPower(power);
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Position", rotation.getCurrentPosition());
            telemetry.update();
        }

    }
}
