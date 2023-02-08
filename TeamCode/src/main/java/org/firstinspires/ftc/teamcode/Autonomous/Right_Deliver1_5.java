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
    public Trajectory driveForward, toPole, toStack, stackToPole, left, right;

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
        driveForward = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(52, 0, Math.toRadians(-90)))
                .build();
        toPole = drive.trajectoryBuilder(driveForward.end(),40, 10)
                .splineToConstantHeading(new Pose2d(56,12),0)
                .build();
        toStack = drive.trajectoryBuilder(driveForward.end().plus(new Pose2d(0, 0, Math.toRadians(-85))), 40, 10)
                .splineToConstantHeading(new Vector2d(52, 0), 0)
                .splineToConstantHeading(new Vector2d(80,0),0)
                .build();
        stackToPole = drive.trajectoryBuilder(toStack.end(), 40, 10)
                .splineToConstantHeading(new Vector2d(52, 0), 0)
                .splineToConstantHeading(new Pose2d(56,12),0)
                .build();
        right = drive.trajectoryBuilder(driveForward.end())
                .strafeRight(24)
                .build();
        left = drive.trajectoryBuilder(driveForward.end())
                .strafeLeft(24)
                .build();
        robot.camera.requestStart();
        robot.grabber.close();
    }

    @Override
    public void onStart() throws InterruptedException {
        parkingPosition = robot.camera.getParkingPosition();
        robot.grabber.close();
        robot.arm.toSideStack();
        drive.followTrajectory(driveForward);
        robot.arm.toHigh();
        drive.followTrajectory(toPole);
        robot.grabber.open();
        for (int i = 5; i > 0; i--) {
            drive.followTrajectory(toStack);
            robot.arm.toSideStack();
            robot.arm.move((int) (0.289 * i * robot.arm.PULSES_PER_REVOLUTION)); // go down height of i cones
            robot.grabber.close();
            robot.arm.toSideStack();
            drive.followTrajectory(stackToPole);
            robot.arm.toHigh();
            robot.grabber.open();
        }
        if (parkingPosition == Camera.ParkingPosition.LEFT) {
            drive.followTrajectory(left);
        } else if (parkingPosition == Camera.ParkingPosition.RIGHT) {
            drive.followTrajectory(right);
        } else {
            // already in the center -> do nothing
        }
    }
}
