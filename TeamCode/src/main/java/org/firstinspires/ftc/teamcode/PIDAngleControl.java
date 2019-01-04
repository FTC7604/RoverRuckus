package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDAngleControl {
    private double error, integral, derivative;
    private double desiredAngle;
    private double lastError = -1;
    private long lastTime = -1;

//    Telemetry telemetry;
//
//    public PIDAngleControl(Telemetry telemetry) {
//        this.telemetry = telemetry;
//    }
//
    public void startPID(double desiredAngle){
        this.desiredAngle = desiredAngle;
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

    public void onSensorChanged(double currentAngle){
        long currentTime = System.currentTimeMillis();
        double elapsedTime = 0.001f * (currentTime - lastTime);

        error = desiredAngle - currentAngle;

        if (lastError != -1) {
            integral = integral + (error * elapsedTime);
            derivative = (error - lastError) / elapsedTime;
        }

        lastTime = currentTime;
        lastError = error;
    }
}