package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

public class PIDAngleControl {
    private double error, integral, derivative;
    private double lastError = -1;
    private long lastTime = -1;

//    Telemetry telemetry;
//
//    public PIDAngleControl(Telemetry telemetry) {
//        this.telemetry = telemetry;
//    }
//
    public void startPID(){
        error = integral = derivative = 0;
        lastError = -1;
        lastTime = -1;
    }

    public double getValue(double kP,double kI, double kD,double mult) {
        //telemetry.addData("P",mult * kP * error);
        //telemetry.addData("I",mult * kI * integral);
        //telemetry.addData("D",mult * kI * derivative);
        //telemetry.update();
        return mult * ((kP * error) + (kI * integral) + (kD * derivative));

    }

    public double getError() {
        return error;
    }

    public double getIntegral() {
        return integral;
    }

    public double getDerivative() {
        return derivative;
    }

    public boolean shouldTerminate(double precision, double maxDifferential)
    {
        return lastTime != -1 && (abs(error) < precision) && (maxDifferential <= 0 || abs(derivative) < maxDifferential);
    }

    public void onSensorChanged(double error) {
        long currentTime = System.currentTimeMillis();
        double elapsedTime = 0.001f * (currentTime - lastTime);

        this.error = error;

        if (lastError != -1) {
            integral = integral + (error * elapsedTime);
            derivative = (error - lastError) / elapsedTime;
        }

        lastTime = currentTime;
        lastError = error;
    }

}