package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PostBot;

@Autonomous
public class GoForward extends BaseOpMode {
    public PostBot robot;

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
    public void onStart() throws InterruptedException {
        robot.pursuit.addPoint(0, 0, 0.0).setLookahead(10d);
//                .addAction(() -> {
//                    robot.arm.toHigh();
//                }, true);
        robot.pursuit.addPoint(0, 24, 0);
        robot.pursuit.move();
    }
}
