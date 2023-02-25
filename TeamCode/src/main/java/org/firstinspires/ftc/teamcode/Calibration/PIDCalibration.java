package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Components.Mecanum;
import org.firstinspires.ftc.teamcode.Components.Odometry;
import org.firstinspires.ftc.teamcode.RMath.Util;
import org.firstinspires.ftc.teamcode.Utils.AverageFilter;
import org.firstinspires.ftc.teamcode.Utils.Button;
import org.firstinspires.ftc.teamcode.Utils.Differentiator;
import org.firstinspires.ftc.teamcode.Utils.PIDController;

import java.io.File;

@TeleOp
@Config
@Disabled
public class PIDCalibration extends BaseOpMode {

//    2 main steps for each tuning mode
//    3 tuning pid
//    1 frictional tuning

//    Different tuning modes:
//    Y position
//    X position
//    Rotation
//    Static friction and kinetic friction

    private Robot driveBot;
    private Odometry odo;
    private Mecanum drivetrain;
    private PIDController forwardPID;
    private Differentiator forwardDiff;

    private PIDController strafePID;
    private Differentiator strafeDiff;

    private PIDController rotPID;
    private Differentiator rotDiff;

    private double staticFrictionForward;
    private double kineticFrictionForward;
    private double staticFrictionStrafe;
    private double kineticFrictionStrafe;
    private double staticFrictionRotation;
    private double kineticFrictionRotation;

    private int buffer = 0;

    private double xPower = 0;
    private double yPower = 0;
    private double angularPower = 0;

    private Button next = new Button();
    private Button down = new Button();
    private Button up = new Button();
    private Button positionToggle = new Button();

    private TuningState tuningState = TuningState.SELECTING;
    private int step = 0;
    private int curserPos = 0;

    public static double forwardTuningDist = 24;
    public static double strafeTuningDist = 24;
    public static double rotTuningDist = 180;
    public static double frictionTuningStep = 0.0005;
    public static double acceleration = 0;

    public Telemetry telemetryy;

    public Robot setRobot() {

        driveBot = new Robot() {
            @Override
            protected void mapHardware(HardwareMap hardwareMap, Telemetry telemetryy, LinearOpMode opMode, boolean isTeleOp) {
                drivetrain = new Mecanum(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight",telemetryy);
                drivetrain.fl.setDirection(DcMotorSimple.Direction.REVERSE);
                drivetrain.fr.setDirection(DcMotorSimple.Direction.FORWARD);
                drivetrain.bl.setDirection(DcMotorSimple.Direction.REVERSE);
                drivetrain.br.setDirection(DcMotorSimple.Direction.FORWARD);

                odo = new Odometry(hardwareMap, "backRight", "frontLeft", "frontRight");
                odo.leftDir = Odometry.EncoderDirection.REVERSE;
                odo.strafeDir = Odometry.EncoderDirection.REVERSE;
                odo.rightDir = Odometry.EncoderDirection.REVERSE;

                addComponents(drivetrain, odo);
            }
        };

        return driveBot;
    }

    @Override
    protected boolean setTeleOp() {
        return false;
    }

    @Override
    public void onInit() {
        telemetryy = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        forwardDiff = new Differentiator(1, 0, true);
        forwardPID = new PIDController(0, 0, 0);
        forwardPID.setPlantSaturation(1);

        strafeDiff = new Differentiator(1, 0, true);
        strafePID = new PIDController(0, 0, 0);
        strafePID.setPlantSaturation(1);

        rotDiff = new Differentiator(1, 0, true);
        rotPID = new PIDController(0, 0, 0);
        rotPID.setPlantSaturation(1);
    }

    @Override
    public void onUpdate() {
        boolean pressed = next.get(gamepad1.a);
        positionToggle.get(gamepad1.b);

        switch (tuningState) {
            case SELECTING:
                String[] options = new String[]{
                        "Friction",
                        "Forward PID",
                        "Strafe PID",
                        "Rotation"
                };


                if (up.get(gamepad1.dpad_up)) curserPos--;
                if (down.get(gamepad1.dpad_down)) curserPos++;

                drivetrain.drive(gamepad1);

                curserPos = (int) Util.loop(curserPos, 0, options.length);
                options[curserPos] += " <";

                telemetryy.addLine("Use 'up' and 'down' to select a mode and press 'a'");
                for (String s : options) {
                    telemetryy.addLine(s);
                }

                if (pressed) {
                    tuningState = TuningState.values()[curserPos + 1];
                    if (tuningState != TuningState.FRICTION) {
                        try {
                            readFrictionValues();
                        }catch (Exception e){
                            telemetryy.clearAll();
                            telemetryy.addLine("Please calibrate friction first");
                            telemetryy.update();
                            sleep(5000);
                            tuningState = TuningState.SELECTING;
                        }
                    }
                    step = 0;
                }

                break;

            case Y_POS: {

                final double forwardPos = odo.getForwardDist();

                switch (step) {
                    case 0:
                        step++;
                        resetEncoders();
//                        Setting the toggle to false
                        if (positionToggle.toggle()) positionToggle.get(true);
                        PIDTuner.kP = 0;
                        PIDTuner.kI = 0;
                        PIDTuner.kD = 0;

                    case 1:
                        telemetryy.addLine("Tune the PID values");
                        telemetryy.addLine("Press 'b' to move to other position");
                        telemetryy.addLine("Press 'a' when finished");

                        final double targetPosition = positionToggle.toggle() ? forwardTuningDist : 0;
                        final double error = targetPosition - forwardPos;

                        telemetryy.addData("Error", error);
                        telemetryy.addData("Y Power", yPower);

                        forwardPID.kP = PIDTuner.kP;
                        forwardPID.kI = PIDTuner.kI;
                        forwardPID.kD = PIDTuner.kD;


                        final double deltaY = Range.clip(forwardPID.getPower(error) * (13 / getBatteryVoltage()) - yPower, -acceleration, acceleration);
                        yPower += deltaY;

                        final double min = (Math.abs(odo.getVelocity().getY()) < 0.2)? staticFrictionForward : ((kineticFrictionForward + staticFrictionForward) * 0.5 + yPower);

                        drivetrain.drive(0, Util.absCap(yPower, min, 1), gamepad1.right_stick_x);

                        if (pressed) step++;
                        break;

                    case 2:
                        telemetryy.addData("kP", forwardPID.kP);
                        telemetryy.addData("kI", forwardPID.kI);
                        telemetryy.addData("kD", forwardPID.kD);
                        telemetryy.addLine("Press 'a' to write to file, press 'b' to cancel");

                        if (pressed) {
                            File file = AppUtil.getInstance().getSettingsFile("forwardPID.txt");
                            ReadWriteFile.writeFile(file, forwardPID.kP + "\n" + forwardPID.kI + "\n" + forwardPID.kD + "\n" + acceleration);
                            tuningState = TuningState.SELECTING;
                            acceleration = 0;
                        }

                        yPower = 0;


                        if (gamepad1.b) {
                            tuningState = TuningState.SELECTING;
                        }
                        break;
                }


                break;
            }

            case X_POS: {

                final double strafePos = odo.getStrafeDist();

                switch (step) {
                    case 0:
                        step++;
                        resetEncoders();
//                        Setting the toggle to false
                        if (positionToggle.toggle()) positionToggle.get(true);
                        PIDTuner.kP = 0;
                        PIDTuner.kI = 0;
                        PIDTuner.kD = 0;

                    case 1:
                        telemetryy.addLine("Tune the PID values");
                        telemetryy.addLine("Press 'b' to move to other position");
                        telemetryy.addLine("Press 'a' when finished");

                        final double targetPosition = positionToggle.toggle() ? strafeTuningDist : 0;
                        final double error = targetPosition - strafePos;
                        telemetryy.addData("Error", error);
                        telemetryy.addData("X Power", xPower);

                        strafePID.kP = PIDTuner.kP;
                        strafePID.kI = PIDTuner.kI;
                        strafePID.kD = PIDTuner.kD;

                        final double deltaX = Range.clip(strafePID.getPower(error) * (13 / getBatteryVoltage()) - xPower, -acceleration, acceleration);
                        xPower += deltaX;

                        final double min = (Math.abs(odo.getVelocity().getX()) < 0.2)? staticFrictionStrafe : kineticFrictionStrafe;

                        drivetrain.drive(Util.absCap(xPower, min, 1), 0, gamepad1.right_stick_x);

                        if (pressed) step++;
                        break;

                    case 2:
                        telemetryy.addData("kP", strafePID.kP);
                        telemetryy.addData("kI", strafePID.kI);
                        telemetryy.addData("kD", strafePID.kD);
                        telemetryy.addLine("Press 'a' to write to file, press 'b' to cancel");

                        xPower = 0;

                        if (pressed) {
                            File file = AppUtil.getInstance().getSettingsFile("strafePID.txt");
                            ReadWriteFile.writeFile(file, strafePID.kP + "\n" + strafePID.kI + "\n" + strafePID.kD + "\n" + acceleration);
                            tuningState = TuningState.SELECTING;
                            acceleration = 0;
                        }

                        if (gamepad1.b) {
                            tuningState = TuningState.SELECTING;
                        }
                        break;
                }


                break;
            }

            case ROT: {

                switch (step) {
                    case 0:

                        step++;
                        resetEncoders();
//                        Setting the toggle to false
                        if (positionToggle.toggle()) positionToggle.get(true);

                    case 1:
                        telemetryy.addLine("Tune the PID values");
                        telemetryy.addLine("Press 'b' to move to other position");
                        telemetryy.addLine("Press 'a' when finished");

                        final double targetPosition = positionToggle.toggle() ? rotTuningDist : 0;
                        final double error = targetPosition - odo.getRotation();
                        telemetryy.addData("Error", error);
                        telemetryy.addData("Turn Power", angularPower);

                        rotPID.kP = PIDTuner.kP;
                        rotPID.kI = PIDTuner.kI;
                        rotPID.kD = PIDTuner.kD;

                        final double deltaRot = Range.clip(-rotPID.getPower(error) * (13 / getBatteryVoltage()) - angularPower, -acceleration, acceleration);
                        angularPower += deltaRot;

                        final double min = (Math.abs(odo.getAngularVelocity()) < 5)? staticFrictionRotation : kineticFrictionRotation;

                        drivetrain.drive(0, 0, Util.absCap(angularPower, min, 1));

                        if (pressed) step++;
                        break;

                    case 2:
                        telemetryy.addData("kP", rotPID.kP);
                        telemetryy.addData("kI", rotPID.kI);
                        telemetryy.addData("kD", rotPID.kD);
                        telemetryy.addLine("Press 'a' to write to file, press 'b' to cancel");

                        if (pressed) {
                            File file = AppUtil.getInstance().getSettingsFile("rotPID.txt");
                            ReadWriteFile.writeFile(file, rotPID.kP + "\n" + rotPID.kI + "\n" + rotPID.kD + "\n" + acceleration);
                            tuningState = TuningState.SELECTING;
                            acceleration = 0;
                        }

                        angularPower = 0;

                        if (gamepad1.b) {
                            tuningState = TuningState.SELECTING;
                        }
                        break;
                }


                break;
            }

            case FRICTION: {

                switch (step) {
                    case 0:
                        telemetryy.addLine("Press 'a' to start tuning forward friction");
                        drivetrain.drive(gamepad1);
                        if (pressed) {
                            step++;
                            yPower = 0;
                            forwardDiff = new Differentiator(1, 0, true);
                            forwardDiff.addFilter(1, new AverageFilter(100));
                            buffer = 0;
                        }
                        break;

                    case 1:
                        yPower += frictionTuningStep;
                        buffer++;
                        forwardDiff.update(odo.getForwardDist());
                        telemetryy.addLine("Tuning forward static friction");
                        telemetryy.addData("Power", yPower);
                        drivetrain.drive(0, yPower * (13 / getBatteryVoltage()), 0);

                        if (forwardDiff.getDerivative(1) > 0.5 && buffer > 100) {
                            staticFrictionForward = yPower;
                            yPower += 0.1;
                            step++;
                        }

                        break;

                    case 2:

                        yPower -= frictionTuningStep;
                        forwardDiff.update(odo.getForwardDist());
                        telemetryy.addLine("Tuning forward kinetic friction");
                        drivetrain.drive(0, yPower * (13 / getBatteryVoltage()), 0);

                        if (forwardDiff.getDerivative(1) <= 0.2) {
                            kineticFrictionForward = yPower;
                            yPower = 0;
                            step++;
                        }

                        break;

                    case 3:
                        telemetryy.addLine("Press 'a' to start tuning strafe friction");
                        drivetrain.drive(gamepad1);
                        if (pressed) {
                            step++;
                            xPower = 0;
                            strafeDiff = new Differentiator(1, 0, true);
                            strafeDiff.addFilter(1, new AverageFilter(100));
                            buffer = 0;
                        }
                        break;

                    case 4:
                        xPower += frictionTuningStep;
                        buffer++;
                        strafeDiff.update(odo.getStrafeDist());
                        telemetryy.addLine("Tuning strafe static friction");
                        telemetryy.addData("Power", xPower);
                        drivetrain.drive(xPower * (13 / getBatteryVoltage()), 0, 0);

                        if (strafeDiff.getDerivative(1) > 0.5 && buffer > 100) {
                            staticFrictionStrafe = xPower;
                            xPower += 0.1;
                            step++;
                        }

                        break;

                    case 5:

                        xPower -= frictionTuningStep;
                        strafeDiff.update(odo.getStrafeDist());
                        telemetryy.addLine("Tuning strafe kinetic friction");
                        drivetrain.drive(xPower * (13 / getBatteryVoltage()), 0, 0);

                        if (strafeDiff.getDerivative(1) <= 0.2) {
                            kineticFrictionStrafe = xPower;
                            xPower = 0;
                            step++;
                        }

                        break;

                    case 6:
                        telemetryy.addLine("Press 'a' to start tuning rotational friction");
                        drivetrain.drive(gamepad1);
                        if (pressed) {
                            step++;
                            angularPower = 0;
                            rotDiff = new Differentiator(1, 0, true);
                            rotDiff.addFilter(1, new AverageFilter(100));
                            buffer = 0;
                        }
                        break;

                    case 7:
                        angularPower += frictionTuningStep;
                        buffer++;
                        rotDiff.update(odo.getRotation());
                        telemetryy.addLine("Tuning rotation static friction");
                        telemetryy.addData("Power", angularPower);
                        telemetryy.addData("Velocity", Math.abs(rotDiff.getDerivative(1)));
                        drivetrain.drive(0, 0, angularPower * (13 / getBatteryVoltage()));

                        if (Math.abs(rotDiff.getDerivative(1)) > 5 && buffer > 100) {
                            staticFrictionRotation = angularPower;
                            angularPower += 0.1;
                            step++;
                        }

                        break;

                    case 8:

                        angularPower -= frictionTuningStep;
                        rotDiff.update(odo.getRotation());
                        telemetryy.addLine("Tuning rotation kinetic friction");
                        drivetrain.drive(0, 0, angularPower * (13 / getBatteryVoltage()));

                        if (Math.abs(rotDiff.getDerivative(1)) <= 2) {
                            kineticFrictionRotation = angularPower;
                            angularPower = 0;
                            step++;
                        }

                        break;

                    case 9:

                        telemetryy.addData("staticFrictionForward", staticFrictionForward);
                        telemetryy.addData("kineticFrictionForward", kineticFrictionForward);
                        telemetryy.addData("staticFrictionStrafe", staticFrictionStrafe);
                        telemetryy.addData("kineticFrictionStrafe", kineticFrictionStrafe);
                        telemetryy.addData("staticFrictionRotation", staticFrictionRotation);
                        telemetryy.addData("kineticFrictionRotation", kineticFrictionRotation);
                        telemetryy.addLine("Press 'a' to write to file, press 'b' to cancel");

                        if (pressed) {
                            File file = AppUtil.getInstance().getSettingsFile("friction.txt");
                            ReadWriteFile.writeFile(file,
                                    staticFrictionForward + "\n" +
                                            kineticFrictionForward + "\n" +
                                            staticFrictionStrafe + "\n" +
                                            kineticFrictionStrafe + "\n" +
                                            staticFrictionRotation + "\n" +
                                            kineticFrictionRotation + "\n"
                            );

                            tuningState = TuningState.SELECTING;
                        }

                }

                break;
            }
        }


        telemetryy.update();
    }

    private void resetEncoders() {
        odo.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo.strafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odo.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odo.strafe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odo.set(0, 0, 0);
    }

    private void readFrictionValues() {
        File file = AppUtil.getInstance().getSettingsFile("friction.txt");
        String[] lines = ReadWriteFile.readFile(file).split("\n");
        staticFrictionForward = Double.parseDouble(lines[0]);
        kineticFrictionForward = Double.parseDouble(lines[1]);
        staticFrictionStrafe = Double.parseDouble(lines[2]);
        kineticFrictionStrafe = Double.parseDouble(lines[3]);
        staticFrictionRotation = Double.parseDouble(lines[4]);
        kineticFrictionRotation = Double.parseDouble(lines[5]);
    }

    public enum TuningState {
        SELECTING,
        FRICTION,
        Y_POS,
        X_POS,
        ROT,
    }
}