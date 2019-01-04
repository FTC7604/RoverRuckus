package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PIDAngleControl;

import java.io.File;

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

        while(opMode.ensureOpModeIsActive() && abs(desiredAngle - position[0]) >= precision)
        {
            position = getIMUPosition();
            pidControl.onSensorChanged(position[0]);
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
