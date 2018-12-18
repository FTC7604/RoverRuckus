package org.firstinspires.ftc.teamcode;

public class PIDAngleControl {
    private double error, integral, derivative;
    private double desiredAngle;
    private double lastError = -1;
    private long lastTime = -1;

    public void startPID(double desiredAngle){
        this.desiredAngle = desiredAngle;
        error = integral = derivative = 0;
        lastError = -1;
        lastTime = -1;
    }

    public double getValue(double kP,double kI, double kD,double mult) {
        return mult * ((kP * error) + (kI * integral) + (kD * derivative));
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