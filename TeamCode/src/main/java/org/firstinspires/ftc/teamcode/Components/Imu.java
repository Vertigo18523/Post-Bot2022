package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Imu implements Component {

    private double heading;
    private double integratedHeading;
    public BNO055IMU imu;
    public Axis axis = Axis.Z;

    private double previousHeading = 0;

    public Imu (HardwareMap map, String name){
        imu = map.get(BNO055IMU.class, name);
    }

    @Override
    public void init() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        update();
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

        AxesOrder order;

        switch (axis){
            case X:
                order = AxesOrder.ZYX;
                break;
            case Y:
                order = AxesOrder.XZY;
                break;
            default:
                order = AxesOrder.XYZ;
        }

        heading = imu.getAngularOrientation(AxesReference.EXTRINSIC, order, AngleUnit.DEGREES).thirdAngle;

        double deltaHeading = heading - previousHeading;
        if (deltaHeading < -180) deltaHeading += 360;
        else if (deltaHeading >= 180) deltaHeading -= 360;

        integratedHeading += deltaHeading;
        previousHeading = heading;
    }

    @Override
    public String getTelemetry() {
        return null;
    }

    public double getHeading(){
        return heading;
    }

    public double getIntegratedHeading() {
        return integratedHeading;
    }

    public enum Axis{
        X,
        Y,
        Z
    }

}