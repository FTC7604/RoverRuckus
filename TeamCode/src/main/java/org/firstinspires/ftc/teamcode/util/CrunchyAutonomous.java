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
    public void turnDegrees(double turnAngle, double precision)
    {
        turnRadians(toRadians(turnAngle), toRadians(precision));
    }

    public void turnRadians(double turnAngle, double precision)
    {
        if (PID_ENABLED)
        {
            turnRadiansPID(turnAngle, precision);
        }
        else
        {
            turnRadiansNoPID(turnAngle, precision);
        }
    }

    private void turnRadiansNoPID(double turnAngle, double precision)
    {
        turnAngle *= -1;

        double[] position = getIMUPosition();
        double desiredAngle = turnAngle + position[0];

        drive(PID_DISABLED_TURN_SPEED, -PID_DISABLED_TURN_SPEED);

        double val;
        while (opMode.ensureOpModeIsActive() && abs(val = getNormalizedError(desiredAngle, position[0])) > precision)
        {
            position = getIMUPosition();
            opMode.telemetry.addData("Desired angle", desiredAngle);
            opMode.telemetry.addData("Position", position[0]);
            opMode.telemetry.addData("Normalized error", val);
            opMode.telemetry.update();
        }

        stopAndResetEncoders();
    }

    private void turnRadiansPID(double turnAngle, double precision)
    {
        turnAngle *= -1;

        double[] position = getIMUPosition();
        double desiredAngle = turnAngle + position[0];
        PIDAngleControl pidControl = new PIDAngleControl();
        pidControl.startPID();

        double pidMult = PID_MULT;

        if(PID_MULT_SCALING)
        {
            pidMult /= (turnAngle / (PI / 4));
        }

        while(opMode.ensureOpModeIsActive() && !pidControl.shouldTerminate(precision, PID_MAX_DIFFERENTIAL))
        {
            position = getIMUPosition();
            pidControl.onSensorChanged(getNormalizedError(desiredAngle, position[0]));
            double turnVal = pidControl.getValue(kP, kI, kD, pidMult);
            turnVal = max(abs(turnVal), PID_MIN_POWER) * signum(turnVal);
            opMode.telemetry.addData("konstants", kP + " " + kI + " " + kD + " " + PID_MULT);
            opMode.telemetry.addData("error", pidControl.getError());
            opMode.telemetry.addData("integral", pidControl.getIntegral());
            opMode.telemetry.addData("derivative", pidControl.getDerivative());
            opMode.telemetry.addData("turn", turnVal);
            opMode.telemetry.update();
            drive(turnVal, -turnVal);
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
