package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PostBot;

@TeleOp
public class MainOp extends BaseOpMode {
    public PostBot robot;
    public double x, y, rot, speed, xModifier, yModifier;
    public boolean slowmode;

    @Override
    protected Robot setRobot() {
        this.robot = new PostBot();
        return this.robot;
    }

    @Override
    protected boolean setTeleOp() {
        return true;
    }

    @Override
    public void onInit() throws InterruptedException {
        robot.grabber.open();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        robot.camera.requestStart();
    }

    @Override
    public void onStart() throws InterruptedException {
//        gamepadListener1.y.onPress = () -> {
//            robot.grabber.close();
//            robot.arm.toHigh();
////            driveForward();
//        };
//        gamepadListener1.a.onPress = () -> {
//            robot.grabber.open();
//            robot.arm.toZero();
////            driveBackward();
//        };

        gamepadListener1.start.onRelease = () -> {
            slowmode = !slowmode;
        };

        gamepadListener2.lb.onPress = () -> {
            robot.grabber.open();
            robot.arm.toZero();
        };
        gamepadListener2.a.onPress = () -> {
            robot.arm.toGround();
        };
        gamepadListener2.dd.onPress = () -> {
            robot.arm.toPickup();
        };
        gamepadListener2.dr.onPress = () -> {
            robot.arm.toSideStack();
        };
        gamepadListener2.b.onPress = () -> {
            robot.arm.toLow();
        };
        gamepadListener2.y.onPress = () -> {
            robot.arm.toMedium();
        };
        gamepadListener2.rb.onPress = () -> {
            robot.arm.toHigh();
        };

        gamepadListener2.x.onRelease = () -> {
            robot.grabber.toggle();
        };

        gamepadListener1.x.onPress = () -> {
            robot.arm.init();
        };

//        gamepadListener2.dr.onPress = () -> {
//            robot.arm.toZero();
//            sleep(100);
//            robot.grabber.close();
//            robot.arm.toLow();
//        };
//
//        gamepadListener2.du.onPress = () -> {
//            robot.arm.toZero();
//            sleep(100);
//            robot.grabber.close();
//            robot.arm.toMedium();
//        };
    }

    @Override
    public void onUpdate() throws InterruptedException {
        speed = (gamepad1.left_bumper ? 0.25 : (gamepad1.right_bumper || slowmode ? 0.5 : 1)) * (gamepad1.left_stick_button ? 1 : 0.65);
        x = gamepad1.left_stick_x;
        y = - gamepad1.left_stick_y;
        rot = gamepad1.right_stick_x;

        if (gamepad1.dpad_up) {
            y = 1;
        } else if (gamepad1.dpad_down) {
            y = - 1;
        }

        if (gamepad1.dpad_right) {
            x = 1;
        } else if (gamepad1.dpad_left) {
            x = - 1;
        }

        if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
//            if (robot.arm.getCurrentPosition() < robot.arm.LOWER_BOUND) {
//                robot.arm.move(robot.arm.LOWER_BOUND + (int) (robot.arm.PULSES_PER_REVOLUTION * 0.014));
//            } else
            if (robot.arm.getCurrentPosition() > robot.arm.UPPER_BOUND) {
                robot.arm.move(robot.arm.UPPER_BOUND - (int) (robot.arm.PULSES_PER_REVOLUTION * 0.014));
            } else {
                robot.arm.move((int) ((gamepad2.right_trigger - gamepad2.left_trigger) * robot.arm.PULSES_PER_REVOLUTION * 0.5) + robot.arm.getCurrentPosition());
            }
        }

        robot.mecanum.drive(x * speed, y * speed, rot * speed);
        robot.arm.getTelemetry();
        robot.rotation.getTelemetry();
        telemetry.update();
    }
}