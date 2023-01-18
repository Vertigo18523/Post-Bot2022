package org.firstinspires.ftc.teamcode.Autonomous.NotDone;

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
        toPole = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12, 21), 0)
                .splineTo(new Vector2d(29, 24), 0)
                .splineTo(new Vector2d(29, 32), 0)
                .splineTo(new Vector2d(34, 36), 0)
                .build();
        forward = drive.trajectoryBuilder(toPole.end())
                .splineTo(new Vector2d(36, 36),0)
                .build();
        backward = drive.trajectoryBuilder(forward.end())
                .splineTo(new Vector2d(30, 36), 0)
                .build();
        left = drive.trajectoryBuilder(backward.end().plus(new Pose2d(0, 0, Math.toRadians(270))))
                .forward(12)
                .build();
        right = drive.trajectoryBuilder(backward.end().plus(new Pose2d(0, 0, Math.toRadians(270))))
                .forward(60)
                .build();
        center = drive.trajectoryBuilder(backward.end().plus(new Pose2d(0, 0, Math.toRadians(270))))
                .forward(36)
                .build();
        robot.camera.requestStart();
        robot.grabber.close();
    }

    @Override
    public void onStart() throws InterruptedException {
        parkingPosition = robot.camera.getParkingPosition();
        state = 0;
        robot.arm.toGround();
        drive.followTrajectory(toPole);
        robot.arm.toHigh();
        drive.followTrajectory(forward);
        robot.arm.toHigh();
        robot.grabber.open();
        drive.followTrajectory(backward);
        robot.arm.toZero();
        drive.turn(Math.toRadians(270));
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
