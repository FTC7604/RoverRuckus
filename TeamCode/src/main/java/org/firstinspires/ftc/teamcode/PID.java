package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

public class PID {
    private double proportionalError, integralError, derivativeError;
    private double lastError = -1;
    private long lastTime = -1;

    public PID(){}

    public void startPID(){
        proportionalError = integralError = derivativeError = 0;
        lastError = -1;
        lastTime = -1;
    }

    public double getValue(double kP,double kI, double kD, double mult) {
        return  mult * (kP * proportionalError) + (kI * integralError) + (kD * derivativeError);

    }

    public double getProportionalError() {
        return proportionalError;
    }

    public double getIntegralError() {
        return integralError;
    }

    public double getDerivativeError() {
        return derivativeError;
    }

    public boolean shouldTerminate(double precision, double maxDifferential) {
        return lastTime != -1 && (abs(proportionalError) < precision) && (maxDifferential <= 0 || abs(derivativeError) < maxDifferential);
    }

    public void onSensorChanged(double error) {
        long currentTime = System.currentTimeMillis();
        double elapsedTime = 0.001f * (currentTime - lastTime);

        this.proportionalError = error;

        if (lastError != -1) {
            integralError = integralError + (error * elapsedTime);
            derivativeError = (error - lastError) / elapsedTime;
        }

        lastTime = currentTime;
        lastError = error;
    }

}