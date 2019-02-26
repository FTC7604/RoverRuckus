package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PIDAngleControl;

import java.io.File;
import static java.lang.Math.*;

public class IMUControl {



    //stuff that is actually used
    public static final double[] compensate(double[] imput, double angle) {
        double x1 = imput[0];
        double y1 = -1 * imput[1];

        double x2 = 0;
        double y2 = 0;

        if (x1 != 0) {
            if (x1 > 0) {
                x2 = sqrt(x1 * x1 + y1 * y1) * cos(atan(y1 / x1) - angle);
                y2 = sqrt(x1 * x1 + y1 * y1) * sin(atan(y1 / x1) - angle);
            } else if (x1 < 0) {
                x2 = sqrt(x1 * x1 + y1 * y1) * cos(PI + atan(y1 / x1) - angle);
                y2 = sqrt(x1 * x1 + y1 * y1) * sin(PI + atan(y1 / x1) - angle);
            }
        } else {
            if (y1 > 0) {
                x2 = abs(y1) * cos(PI / 2 - angle);
                y2 = abs(y1) * sin(PI / 2 - angle);
            } else if (y1 < 0) {
                x2 = abs(y1) * cos(3 * PI / 2 - angle);
                y2 = abs(y1) * sin(3 * PI / 2 - angle);
            } else {
                x2 = 0;
                y2 = 0;
            }
        }

        imput[0] = x2;
        imput[1] = -1 * y2;

        return imput;
    }
    public static final double getYaw(double oldYaw, BNO055IMU imu1, BNO055IMU imu2){

        double Yaw = imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle;

        if (Yaw - oldYaw > PI) Yaw -= 2 * PI;
        if (oldYaw - Yaw > PI) Yaw += 2 * PI;


        return Yaw;
    }

    //stuff that is cool but is not used becuase it causes latency
    private double positiveAngle(double angle) {
        /*All that this method does is convert this angle to something between 0 and 2PI, I don't want the robot to keep spinning around and I wouldn't be able to
        understand the meaning of the huge value. */

        //while the angle is less than 0, add 2 PI
        while (angle < 0) {
            angle += 2 * PI;
        }
        //while its greater than 2PI, subtract PI
        while (angle >= 2 * PI) {
            angle -= 2 * PI;
        }

        return angle;
    }
    private double[] smooth(double[] smoothData, double[] newData, double fraction, boolean restrict) {

        for (int axis = 2; axis > -1; axis--) {
            //smooths the data
            smoothData[axis] = ((1 - fraction) * smoothData[axis]) + (fraction * newData[axis]);
            //prevents the data from remaining within the bounds
            if (restrict) {
                smoothData[0] = positiveAngle(smoothData[0]);
            }
        }

        return smoothData;

    }
    public double[] getPosition(double[] outputPosition, BNO055IMU imu1, BNO055IMU imu2, boolean restrict) {
        double[] imputPosition = new double[3];

        //gets the raw values.
        imputPosition[0] = (imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle) / 2;//Yaw
        imputPosition[1] = (imu1.getAngularOrientation().secondAngle + imu2.getAngularOrientation().secondAngle) / 2;//Roll
        imputPosition[2] = (imu1.getAngularOrientation().thirdAngle + imu2.getAngularOrientation().thirdAngle) / 2;//Pitch

        //The angles are on the circle can't just be averaged, so if the difference, is greater than pi, then it changes the value so it isn't
        for (int axis = 2; axis > 0 - 1; axis--) {
            if (imputPosition[axis] - outputPosition[axis] > PI) imputPosition[axis] -= 2 * PI;
            if (outputPosition[axis] - imputPosition[axis] > PI) imputPosition[axis] += 2 * PI;
        }

        smooth(outputPosition, imputPosition, .8, restrict);

        return outputPosition;
    }
}