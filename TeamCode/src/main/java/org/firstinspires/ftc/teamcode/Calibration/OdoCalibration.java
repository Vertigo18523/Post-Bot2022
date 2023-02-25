package org.firstinspires.ftc.teamcode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Components.Imu;
import org.firstinspires.ftc.teamcode.Components.Mecanum;
import org.firstinspires.ftc.teamcode.RMath.Util;

@Autonomous(group="Odometry")
@Disabled
public class OdoCalibration extends LinearOpMode {

    private boolean LEFT_REVERSED = true;
    private boolean RIGHT_REVERSED = true;
    private boolean STRAFE_REVERSED = true;

    @Override

    public void runOpMode() {

        Mecanum drivetrain = new Mecanum(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight", telemetry);
        Imu imu = new Imu(hardwareMap, "imu");
        imu.axis = Imu.Axis.X;

        DcMotor left = hardwareMap.dcMotor.get("backRight");
        DcMotor strafe = hardwareMap.dcMotor.get("frontLeft");
        DcMotor right = hardwareMap.dcMotor.get("backLeft");

        File ticksPerDegreeFile = AppUtil.getInstance().getSettingsFile("odoTicksPerDegree.txt");
        File strafeTicksPerDegreeFile = AppUtil.getInstance().getSettingsFile("odoStrafeTicksPerDegree.txt");

        final int leftInitialPosition = left.getCurrentPosition();
        final int strafeInitialPosition = strafe.getCurrentPosition();
        final int rightInitialPosition = right.getCurrentPosition();

        drivetrain.init();
        imu.init();

        while(!opModeIsActive()){
            telemetry.addData("heading", imu.getHeading());
            telemetry.update();
        }

        final double initialHeading = imu.getHeading();

//        Turn the robot approx. 90 degrees
        for(double h = 0; Math.abs(getAngleDiff(h, initialHeading)) < 90; h = imu.getHeading()) {
            if(!opModeIsActive()) return;

            imu.update();

            double error = Util.absCap(getAngleDiff(90, h - initialHeading) / 45, 0.1, 0.4);
            drivetrain.move(-error, error, -error, error);
            drivetrain.update();

            telemetry.addData("Heading", h);
            telemetry.addData("Left wheel position", left.getCurrentPosition());
            telemetry.addData("Strafe wheel position", strafe.getCurrentPosition());
            telemetry.addData("Right wheel position", right.getCurrentPosition());
            telemetry.update();
        }

        drivetrain.move(0,0,0,0);
        drivetrain.update();

//        Wait for the robot to slow down to a stop
        sleep(1000);

//        Finding the change in position of all the encoders
        final int leftDeltaPosition = (left.getCurrentPosition() - leftInitialPosition) * (LEFT_REVERSED? -1 : 1);
        final int strafeDeltaPosition = (strafe.getCurrentPosition() - strafeInitialPosition) * (STRAFE_REVERSED? -1 : 1);
        final int rightDeltaPosition = (right.getCurrentPosition() - rightInitialPosition) * (RIGHT_REVERSED? -1 : 1);

//        The change in heading
        final double deltaHeading = getAngleDiff(imu.getHeading(), initialHeading);

        final double turnDeltaPosition = Math.abs(rightDeltaPosition - leftDeltaPosition) / 2.0;

//        These are the ratios that we need to store in the files, so we can use them for calculating
//        our localized position
        double ticksPerDegree = Math.abs(turnDeltaPosition / deltaHeading);
        double strafeTicksPerDegree = Math.abs(strafeDeltaPosition / deltaHeading);

//        Writing to the files so they can be used
        ReadWriteFile.writeFile(ticksPerDegreeFile, Double.toString(ticksPerDegree));
        ReadWriteFile.writeFile(strafeTicksPerDegreeFile, Double.toString(strafeTicksPerDegree));

        ticksPerDegree = Double.parseDouble(ReadWriteFile.readFile(ticksPerDegreeFile).trim());
        strafeTicksPerDegree = Double.parseDouble(ReadWriteFile.readFile(strafeTicksPerDegreeFile).trim());

        while(opModeIsActive()){
            telemetry.addData("ticksPerDegree", ticksPerDegree);
            telemetry.addData("strafeTicksPerDegree", strafeTicksPerDegree);
            telemetry.addData("turnDeltaPosition", turnDeltaPosition);
            telemetry.addData("deltaHeading", deltaHeading);
            telemetry.update();
        }

    }

    private double getAngleDiff(double a1, double a2) {
        a1 = Util.loop(a1, 0, 360);
        a2 = Util.loop(a2, 0, 360);

        double dist = a1 - a2;
        double shortest;
        if (Math.abs(dist) < 180)
            shortest = dist;
        else {
            if (dist > 0) shortest = dist - 360;
            else shortest = dist + 360;
        }

        return shortest;
    }

}