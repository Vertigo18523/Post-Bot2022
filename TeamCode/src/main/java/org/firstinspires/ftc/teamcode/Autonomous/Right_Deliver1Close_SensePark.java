package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PostBot;
import org.firstinspires.ftc.teamcode.Components.Camera;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RRMecanum;

@Autonomous
public class Right_Deliver1Close_SensePark extends BaseOpMode {
    public PostBot robot;
    public RRMecanum drive;
    public Trajectory toPole, forward, backward, left, right, center;

    Camera.ParkingPosition parkingPosition;

    public int state = 100;

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
        drive = new RRMecanum(hardwareMap);
        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);
        toPole = drive.trajectoryBuilder(startPose, 20,3)
                .splineTo(new Vector2d(12, 20), 0)
                .splineTo(new Vector2d(29, 24), 0)
                .splineToConstantHeading(new Vector2d(29, 32), 0)
                .splineToConstantHeading(new Vector2d(31, 36.5), 0)
                .build();
        forward = drive.trajectoryBuilder(toPole.end(),20,3)
                .splineTo(new Vector2d(33.5, 36.5),0)
                .build();
        backward = drive.trajectoryBuilder(forward.end())
                .back(7.5)
                .build();
        left = drive.trajectoryBuilder(backward.end().plus(new Pose2d(0, 0, Math.toRadians(0))),30,3)
                //.splineToConstantHeading(new Vector2d(26,27), 0)
                .strafeRight(9.5)
                .build();
        right = drive.trajectoryBuilder(backward.end().plus(new Pose2d(0, 0, Math.toRadians(0))),30,3)
                //.splineToConstantHeading(new Vector2d(26,-24), 0)\
                .strafeRight(60.5)
                .build();
        center = drive.trajectoryBuilder(backward.end().plus(new Pose2d(0, 0, Math.toRadians(0))),30,3)
                //.splineToConstantHeading(new Vector2d(26,0), 0)
                .strafeRight(36.5)
                .build();
        robot.camera.requestStart();
        robot.grabber.close();
    }

    @Override
    public void onStart() throws InterruptedException {
        parkingPosition = robot.camera.getParkingPosition();
        state = 0;
        robot.arm.toGround();
        robot.grabber.close();
        drive.followTrajectory(toPole);
        robot.arm.toHigh();
        drive.followTrajectory(forward);
        robot.arm.toHigh();
        robot.grabber.open();
        sleep(2000);
        drive.followTrajectory(backward);
        robot.arm.toZero();
        if (parkingPosition == Camera.ParkingPosition.LEFT) {
            drive.followTrajectory(left);
        } else if (parkingPosition == Camera.ParkingPosition.RIGHT) {
            drive.followTrajectory(right);
        } else {
            drive.followTrajectory(center);
        }
    }

//    @Override
//    public void onUpdate() throws InterruptedException {
//        switch (state) {
//            case 0:
//                if (!drive.isBusy()) {
//                    drive.followTrajectoryAsync(toPole);
//                    state++;
//                }
//                break;
//            case 1:
//                state++;
//                robot.arm.toHigh();
//                break;
//            case 2:
//                break;
//            case 100:
//                break;
//        }
//        drive.update();
//    }
}
