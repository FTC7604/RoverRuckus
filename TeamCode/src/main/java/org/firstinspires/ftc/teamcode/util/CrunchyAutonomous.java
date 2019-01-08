package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.PIDAngleControl;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
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


    public void strafeRightForDistance(double power, int distance)
    {
        stopAndResetEncoders();
        power *= signum(distance);
        distance = abs(distance);

        double x = -power * abs(power);

        double currentPosition;
        while (opMode.ensureOpModeIsActive() && (currentPosition = getDriveEncoderValue()) < distance)
        {
            double remaining = distance - currentPosition;
            double ratio = (remaining + distance) / (2.0 * distance);
            double val = x * ratio;
            drive(-val, val, val, -val);
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
        turnAngle *= -1;

        double[] position = getIMUPosition();
        double desiredAngle = turnAngle + position[0];
        PIDAngleControl pidControl = new PIDAngleControl();
        pidControl.startPID(desiredAngle);

        while(opMode.ensureOpModeIsActive() && !pidControl.shouldTerminate(precision))
        {
            position = getIMUPosition();
            pidControl.newErrorValue(position[0]);
            double turnVal = pidControl.getValue(kP, kI, kD, pidMult);
            opMode.telemetry.addData("konstants", kP + " " + kI + " " + kD + " " + pidMult);
            opMode.telemetry.addData("error", pidControl.getError());
            opMode.telemetry.addData("integral", pidControl.getIntegral());
            opMode.telemetry.addData("derivative", pidControl.getDerivative());
            opMode.telemetry.addData("turn", turnVal);
            opMode.telemetry.update();
            drive(turnVal, -turnVal);
        }

        stopAndResetEncoders();
    }
}
