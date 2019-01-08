package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.PI;

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

    public boolean shouldTerminate(double precision)
    {
        return lastTime != -1 && (error < precision);
    }

    public void newErrorValue(double currentAngle) {
        long currentTime = System.currentTimeMillis();
        double elapsedTime = 0.001f * (currentTime - lastTime);

        error = getPIDError(currentAngle);

        if (lastError != -1) {
            integral = integral + (error * elapsedTime);
            derivative = (error - lastError) / elapsedTime;
        }

        lastTime = currentTime;
        lastError = error;
    }

    private double getPIDError(double value)
    {
        double error = desiredAngle - value;
        if(error < -PI)
        {
            return error + (2 * PI);
        }
        else if (error > PI)
        {
            return error - (2 * PI);
        }
        else
        {
            return error;
        }
    }
}