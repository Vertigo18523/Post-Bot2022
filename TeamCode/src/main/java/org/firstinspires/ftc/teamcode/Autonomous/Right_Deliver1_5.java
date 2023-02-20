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
    public Trajectory driveForward, toPole, toStack, stackToPole, left, center, right;

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
        driveForward = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(52, 0, Math.toRadians(-90)))
                .addDisplacementMarker(() -> robot.arm.toHigh())
                .build();
        toPole = drive.trajectoryBuilder(driveForward.end(),40, 10)
                .splineToConstantHeading(new Vector2d(50,24),0)
                .splineToConstantHeading(new Vector2d(58,25),0)
                .addDisplacementMarker(() -> robot.grabber.open())
                .build();
        toStack = drive.trajectoryBuilder(driveForward.end().plus(new Pose2d(0, 0, Math.toRadians(0))), 40, 10)
                .splineToConstantHeading(new Vector2d(58,24),0)
                .splineToConstantHeading(new Vector2d(52,24),0)
                .splineToConstantHeading(new Vector2d(52, 0), 0)
                .splineToConstantHeading(new Vector2d(52,-28),0)
                .addDisplacementMarker(() -> {
                    robot.arm.toSideStack();
                    robot.arm.move((int) (0.289 * i * robot.arm.PULSES_PER_REVOLUTION)); // go down height of i cones
                    //rotation arm stuff is going to go here
                    robot.grabber.close();
                    robot.arm.toSideStack();
                })
                .build();
        stackToPole = drive.trajectoryBuilder(toStack.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), 40, 10)
                .splineToConstantHeading(new Vector2d(52, 0), 0)
                .splineToConstantHeading(new Vector2d(58,24),0)
                .addDisplacementMarker(() -> {
                    robot.arm.toHigh();
                    //more rotation arm stuff here
                    robot.grabber.open();
                })
                .build();
        right = drive.trajectoryBuilder(stackToPole.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .splineToConstantHeading(new Vector2d(54, 24), 0)
                .splineToConstantHeading(new Vector2d(54, -36), 0)
                .build();
        center =  drive.trajectoryBuilder(stackToPole.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .splineToConstantHeading(new Vector2d(54, 24), 0)
                .splineToConstantHeading(new Vector2d(54, -12), 0)
                .build();
        left = drive.trajectoryBuilder(stackToPole.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .splineToConstantHeading(new Vector2d(54, 24), 0)
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
        drive.followTrajectory(toPole);
        for (i = 5; i > 0; i--) {
            drive.followTrajectory(toStack);
            drive.followTrajectory(stackToPole);
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
