package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PostBot;
import org.firstinspires.ftc.teamcode.Components.Camera;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RRMecanum;

@Autonomous
public class Park extends BaseOpMode {
    public PostBot robot;
    public RRMecanum drive;
    public Trajectory left, right, center;

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
        drive = new RRMecanum(hardwareMap);
        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);
        left = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12, 21), 0)
                .splineTo(new Vector2d(29, 24), 0)
                .build();
        right = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12, -21), 0)
                .splineTo(new Vector2d(29, -24), 0)
                .build();
        center = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(29, 0), 0)
                .build();
        robot.camera.requestStart();
        robot.grabber.close();
    }

    @Override
    public void onStart() throws InterruptedException {
        parkingPosition = robot.camera.getParkingPosition();

        if (parkingPosition == Camera.ParkingPosition.LEFT) {
            drive.followTrajectory(left);
        } else if (parkingPosition == Camera.ParkingPosition.RIGHT) {
            drive.followTrajectory(right);
        } else {
            drive.followTrajectory(center);
        }
    }
}