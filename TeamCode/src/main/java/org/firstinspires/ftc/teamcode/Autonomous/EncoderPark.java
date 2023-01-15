package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PostBot;
import org.firstinspires.ftc.teamcode.Components.Camera;

@Autonomous
@Disabled
public class EncoderPark extends BaseOpMode {
    public PostBot robot;

    Camera.ParkingPosition parkingPosition;

    @Override
    protected Robot setRobot() {
        this.robot = new PostBot();
        return this.robot;
    }

    @Override
    protected boolean setTeleOp() {
        return false;
    }

    @Override
    public void onInit() throws InterruptedException {
        robot.camera.requestStart();
        robot.grabber.close();
    }

    @Override
    public void onStart() throws InterruptedException {
        parkingPosition = robot.camera.getParkingPosition();

        if (parkingPosition == Camera.ParkingPosition.LEFT) {
            robot.encoderMecanum.strafeLeft(24);
        } else if (parkingPosition == Camera.ParkingPosition.RIGHT) {
            robot.encoderMecanum.strafeRight(24);
        }
        robot.encoderMecanum.driveForward(29);
    }
}