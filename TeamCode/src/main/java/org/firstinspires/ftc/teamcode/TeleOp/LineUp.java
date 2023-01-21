package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PostBot;

@TeleOp
public class LineUp extends MainOp {
    @Override
    public void onInit() throws InterruptedException {
        super.onInit();
        FtcDashboard.getInstance().startCameraStream(robot.camera.requestStart(), 0);
    }
}
