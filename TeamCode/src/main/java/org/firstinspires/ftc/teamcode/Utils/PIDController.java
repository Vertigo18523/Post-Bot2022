package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.RMath.Util;

public class PIDController {

    public Differentiator error;

    public double kP;
    public double kI;
    public double kD;

    private Double plantSaturation = Double.POSITIVE_INFINITY;

    public PIDController(double kP, double kI, double kD){
        error = new Differentiator(1,1, true);
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getPower(double error){
        this.error.update(error);

        double p = kP * this.error.getValue();
        double i = kI * this.error.getIntegral(1);
        if(plantSaturation != null){
            i = Util.cap(i, -plantSaturation, plantSaturation);
        }
        double d = kD * this.error.getDerivative(1);

        return p + i + d;
    }

    public void setPlantSaturation(double d){
        plantSaturation = d;
    }

    public double getPlantSaturation(){
        return plantSaturation;
    }

}