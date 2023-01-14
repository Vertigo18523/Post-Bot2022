package org.firstinspires.ftc.teamcode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PostBot;
import org.firstinspires.ftc.teamcode.Components.Mecanum;


@TeleOp(group = "calibration")
public class OdoTelemetryTeleOp extends BaseOpMode {

    private PostBot driveBot;


    private DcMotor left;
    private DcMotor strafe;
    private DcMotor right;

    @Override
    public Robot setRobot() {
        driveBot = new PostBot();
        return driveBot;
    }

    @Override
    protected boolean setTeleOp() {
        return true;
    }

    @Override
    public void onInit() {

        // Change these because they are probably not up to date with turret bot
        left = hardwareMap.dcMotor.get("backRight");
        strafe = hardwareMap.dcMotor.get("frontLeft");
        right = hardwareMap.dcMotor.get("frontRight");

    }
    @Override
    public void onStart() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void onUpdate() {
        telemetry.addData("left", left.getCurrentPosition());
        telemetry.addData("strafe", strafe.getCurrentPosition());
        telemetry.addData("right", right.getCurrentPosition());
        telemetry.update();
        driveBot.mecanum.drive(gamepad1.left_stick_x / 2, - gamepad1.left_stick_y / 2, gamepad1.right_stick_x / 2);

    }
}

