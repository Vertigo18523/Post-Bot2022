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
public class Right_Deliver1_5_V2 extends BaseOpMode {
    public PostBot robot;
    public RRMecanum drive;
    public Trajectory driveForward, toPole1, toPole2, toStack1, toStack2, stackToPole1, stackToPole2, left, center, right;

    Camera.ParkingPosition parkingPosition;

    public int i;

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
        driveForward = drive.trajectoryBuilder(startPose,20,10)
                .lineToSplineHeading(new Pose2d(50, 0, Math.toRadians(-90)))
                .build();
        toPole1 = drive.trajectoryBuilder(driveForward.end(),20, 10)
                .splineToConstantHeading(new Vector2d(50,26),0)
                .splineToConstantHeading(new Vector2d(60,26),0)
                .build();
        toPole2 = drive.trajectoryBuilder(toPole1.end(),20, 10)
                .splineToConstantHeading(new Vector2d(60,20),0)
                .build();
        toStack1 = drive.trajectoryBuilder(toPole2.end(), 20, 10)
                .splineToConstantHeading(new Vector2d(60,26),0)
                .build();
        toStack2 = drive.trajectoryBuilder(toPole2.end(), 20, 10)
                .splineToConstantHeading(new Vector2d(50,26),0)
                .splineToConstantHeading(new Vector2d(50,-28),0)
                .build();
        stackToPole1 = drive.trajectoryBuilder(toStack2.end(), 30, 10)
                .splineToConstantHeading(new Vector2d(50, 0), 0)
                .splineToConstantHeading(new Vector2d(50,26),0)
                .splineToConstantHeading(new Vector2d(60,26),0)
                .build();
        stackToPole2 = drive.trajectoryBuilder(stackToPole1.end(), 30, 10)
                .splineToConstantHeading(new Vector2d(60,20),0)
                .build();
        right = drive.trajectoryBuilder(stackToPole2.end(),30,10)
                .splineToConstantHeading(new Vector2d(50, 26), 0)
                .splineToConstantHeading(new Vector2d(50, -36), 0)
                .build();
        center =  drive.trajectoryBuilder(stackToPole2.end(),30,10)
                .splineToConstantHeading(new Vector2d(50, 26), 0)
                .splineToConstantHeading(new Vector2d(50, -12), 0)
                .build();
        left = drive.trajectoryBuilder(stackToPole2.end(),30,10)
                .splineToConstantHeading(new Vector2d(50, 26), 0)
                .build();
        robot.camera.requestStart();
        robot.grabber.close();
    }

    @Override
    public void onStart() throws InterruptedException {
        parkingPosition = robot.camera.getParkingPosition();
        robot.grabber.close();
        robot.arm.toSideStack();
        drive.followTrajectoryAsync(driveForward);
        drive.followTrajectoryAsync(toPole1);
        robot.arm.toHigh();
        robot.rotation.toForward();
        drive.followTrajectoryAsync(toPole2);
        robot.grabber.open();
        for (i = 5; i > 0; i--) {
            drive.followTrajectoryAsync(toStack1);
            robot.rotation.move(0);
            robot.arm.toSideStack();
            drive.followTrajectoryAsync(toStack2);
            robot.arm.toSideStack();
            robot.rotation.toForward();
            robot.arm.move((int) (0.289 * i * robot.arm.PULSES_PER_REVOLUTION)); // go down height of i cones
            robot.grabber.close();
            robot.rotation.move(0);
            robot.arm.toSideStack();

            drive.followTrajectoryAsync(stackToPole1);
            robot.arm.toHigh();
            robot.rotation.toForward();
            drive.followTrajectoryAsync(stackToPole2);
            robot.grabber.open();
        }
        if (parkingPosition == Camera.ParkingPosition.LEFT) {
            drive.followTrajectoryAsync(left);
        } else if (parkingPosition == Camera.ParkingPosition.RIGHT) {
            drive.followTrajectoryAsync(right);
        } else {
            drive.followTrajectoryAsync(center);
        }
    }

    @Override
    public void onUpdate() throws InterruptedException {
        drive.update();
    }
}