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
public class Right_Deliver1_5 extends BaseOpMode {
    public PostBot robot;
    public RRMecanum drive;
    public Trajectory toPole, forward, backward, coneStack, left, right, center;

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
        toPole = drive.trajectoryBuilder(startPose, 20, 3)
                .splineTo(new Vector2d(60, 0), Math.toRadians(0))
                .build();
        forward = drive.trajectoryBuilder(toPole.end().plus(new Pose2d(0, 0, Math.toRadians(90))), 20, 3)
                .forward(1)
                .build();
        backward = drive.trajectoryBuilder(forward.end())
                .back(1)
                .build();
        coneStack = drive.trajectoryBuilder(backward.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .forward(12)
                .build();
        left = drive.trajectoryBuilder(backward.end().plus(new Pose2d(0, 0, Math.toRadians(90))), 30, 3)
                .splineToConstantHeading(new Vector2d(38, 0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(38, 38), Math.toRadians(0))
                .build();
        right = drive.trajectoryBuilder(backward.end().plus(new Pose2d(0, 0, Math.toRadians(90))), 30, 3)
                .splineToConstantHeading(new Vector2d(38, 0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(38, -12), Math.toRadians(0))
                .build();
        center = drive.trajectoryBuilder(backward.end().plus(new Pose2d(0, 0, Math.toRadians(90))), 30, 3)
                .splineToConstantHeading(new Vector2d(38, 0), Math.toRadians(0))
                .build();
        robot.camera.requestStart();
        robot.grabber.close();
    }

    @Override
    public void onStart() throws InterruptedException {
        parkingPosition = robot.camera.getParkingPosition();
        state = 0;
        drive.followTrajectory(toPole);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(forward);
        drive.followTrajectory(backward);
        drive.turn(Math.toRadians(180) - 1e-6);
        for (int i = 5; i > 0; i--) {
            robot.arm.toSideStack();
            drive.followTrajectory(coneStack);
            drive.followTrajectory(forward);
            robot.arm.move((int) (0.289 * i * robot.arm.PULSES_PER_REVOLUTION)); // go down height of i cones
            robot.grabber.close();
            robot.arm.toSideStack();
            drive.followTrajectory(backward);
            drive.turn(Math.toRadians(180) + 1e-6);
            drive.followTrajectory(coneStack);
            robot.arm.toHigh();
            drive.followTrajectory(forward);
            robot.grabber.open();
            drive.followTrajectory(backward);
        }
        if (parkingPosition == Camera.ParkingPosition.LEFT) {
            drive.followTrajectory(left);
        } else if (parkingPosition == Camera.ParkingPosition.RIGHT) {
            drive.followTrajectory(right);
        } else {
            drive.followTrajectory(center);
        }
    }
}
