package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.PIDAngleControl;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.signum;
import static java.lang.Math.toRadians;

public class CrunchyAutonomous extends Crunchy
{
    private DWAILinearOpMode opMode;

    public CrunchyAutonomous(DWAILinearOpMode opMode)
    {
        super(opMode);
        this.opMode = opMode;
    }

    public void driveForwardForTime(double power, long time)
    {
        power *= signum(time);
        time = abs(time);

        drive(power);
        opMode.sleep(time);
        stopAndResetEncoders();
    }

    public void driveForwardForDistance(double power, int distance)
    {
        stopAndResetEncoders();
        power *= signum(distance);
        distance = abs(distance);

        double currentPosition;
        while (opMode.ensureOpModeIsActive() && (currentPosition = getDriveEncoderValue()) < distance)
        {
            double remaining = distance - currentPosition;
            double ratio = (remaining + distance) / (2.0 * distance);
            drive(power * ratio);
        }
        stopAndResetEncoders();
    }

    public void strafeRight(double power)
    {
        double x = -power * abs(power);
        drive(-x, x, x, -x);
    }

    public void strafeRightForTime(double power, long time)
    {
        power *= signum(time);
        time = abs(time);

        strafeRight(power);
        opMode.sleep(time);
        stopAndResetEncoders();
    }

    public void strafeRightForDistance(double power, int distance)
    {
        stopAndResetEncoders();
        power *= signum(distance);
        distance = abs(distance);

        double currentPosition;
        while (opMode.ensureOpModeIsActive() && (currentPosition = getDriveEncoderValue()) < distance)
        {
            double remaining = distance - currentPosition;
            double ratio = (remaining + distance) / (2.0 * distance);
            strafeRight(power * ratio);
        }
        stopAndResetEncoders();
    }

    /* Positive turn direction is right (clockwise) */
    public void turnDegrees(double power, double turnAngle, double precision)
    {
        turnRadians(power, toRadians(turnAngle), toRadians(precision));
    }

    public void turnRadians(double power, double turnAngle, double precision)
    {
        turnAngle *= -1;

        double[] position = getIMUPosition();
        double desiredAngle = turnAngle + position[0];

        PIDAngleControl pidControl = new PIDAngleControl();
        if(PID_ENABLED)
        {
            pidControl.startPID();
        }

        double error;
        while (opMode.ensureOpModeIsActive() && abs(error = getNormalizedError(desiredAngle, position[0])) > (precision * PID_PRECISION_THRESHOLD_MULT))
        {
            position = getIMUPosition();
            opMode.telemetry.addData("Desired angle", desiredAngle);
            opMode.telemetry.addData("Position", position[0]);
            opMode.telemetry.addData("Normalized error", error);
            opMode.telemetry.update();

            double ratio = signum(error) * ((abs(error) + abs(turnAngle)) / (2.0 * abs(turnAngle)));
            drive(-power * ratio, power * ratio);
        }

        if(PID_ENABLED)
        {
            while(opMode.ensureOpModeIsActive() && !pidControl.shouldTerminate(precision, PID_MAX_DIFFERENTIAL))
            {
                position = getIMUPosition();
                pidControl.onSensorChanged(getNormalizedError(desiredAngle, position[0]));
                double turnVal = pidControl.getValue(kP, kI, kD, PID_MULT);
                turnVal = max(abs(turnVal), PID_MIN_POWER) * signum(turnVal);
                opMode.telemetry.addData("konstants", kP + " " + kI + " " + kD + " " + PID_MULT);
                opMode.telemetry.addData("error", pidControl.getError());
                opMode.telemetry.addData("integral", pidControl.getIntegral());
                opMode.telemetry.addData("derivative", pidControl.getDerivative());
                opMode.telemetry.addData("turn", turnVal);
                opMode.telemetry.update();
                drive(-turnVal, turnVal);
            }
        }

        stopAndResetEncoders();
    }

    private double getNormalizedError(double desiredAngle, double value)
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
