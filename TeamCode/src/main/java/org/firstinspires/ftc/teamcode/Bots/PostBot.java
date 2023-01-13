package org.firstinspires.ftc.teamcode.Bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.AutoMecanum;
import org.firstinspires.ftc.teamcode.Components.Camera;
import org.firstinspires.ftc.teamcode.Components.EncoderMecanum;
import org.firstinspires.ftc.teamcode.Components.Grabber;
import org.firstinspires.ftc.teamcode.Components.Mecanum;
import org.firstinspires.ftc.teamcode.Components.Odometry;
import org.firstinspires.ftc.teamcode.Components.PIDMecanum;
import org.firstinspires.ftc.teamcode.Components.PurePursuit;

public class PostBot extends Robot {
    public boolean isTeleOp;
    public Camera camera;
    public Arm arm;
    public Grabber grabber;
    public Mecanum mecanum;
//    public EncoderMecanum encoderMecanum;
    public AutoMecanum encoderMecanum;
    public PIDMecanum pidMecanum;
    public Odometry odo;
    public PurePursuit pursuit;

    @Override
    protected void mapHardware(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, boolean isTeleOp) {
        this.isTeleOp = isTeleOp;
        this.camera = new Camera(opMode, "Webcam 1", hardwareMap, telemetry);
        this.arm = new Arm(
                "arm",
                "leftArm",
                hardwareMap,
                telemetry,
                isTeleOp,
                -0.026,
                0.096,
                0.388,
                0.975,
                3.277,
                5.462,
                7.672,
                7.802
        );
        this.grabber = new Grabber(opMode, "grabber", hardwareMap, telemetry);
        if (isTeleOp) {
            this.mecanum = new Mecanum(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight", telemetry);
            addComponents(mecanum);
        } else {
            /*
                left: backRight
                right: frontRight
                strafe: frontLeft
             */
//            this.encoderMecanum =
//                    new AutoMecanum(
//                            opMode,
//                            "frontLeft",
//                            "frontRight",
//                            "backLeft",
//                            "backRight",
//                            hardwareMap,
//                            telemetry,
//                            false,
//                            0.8,
//                            0.5,
//                            10.5,
//                            12.5,
//                            1.1,
//                            100,
//                            false,
//                            0,
//                            0,
//                            0,
//                            arm
//                    );
//            addComponents(encoderMecanum);
            this.odo = new Odometry(hardwareMap, "backRight", "frontLeft", "frontRight");
            odo.leftDir = Odometry.EncoderDirection.FORWARD;
            odo.strafeDir = Odometry.EncoderDirection.FORWARD;
            odo.rightDir = Odometry.EncoderDirection.FORWARD;
            this.pidMecanum = new PIDMecanum(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight", odo);
            this.pursuit = new PurePursuit(pidMecanum, odo);
            addComponents(pursuit, pidMecanum);
        }

        addComponents(camera, grabber, arm);
    }
}
