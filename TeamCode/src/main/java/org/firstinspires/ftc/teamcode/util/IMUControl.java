package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PID;

import java.io.File;
import static java.lang.Math.*;

public class IMUControl {

    private Telemetry telemetry;
    private PID PIDControl = new PID();

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
            if(restrict) {
                smoothData[0] = positiveAngle(smoothData[0]);
            }
        }

        return smoothData;

    }

    public double[] compensate(double[] imput, double angle) {
        double x1 = imput[0];
        double y1 = -1 * imput[1];

        double x2 = 0;
        double y2 = 0;

        if(x1 != 0) {
            if(x1 > 0) {
                x2 = sqrt(x1 * x1 + y1 * y1) * cos(atan(y1 / x1) - angle);
                y2 = sqrt(x1 * x1 + y1 * y1) * sin(atan(y1 / x1) - angle);
            }
            else if(x1 < 0) {
                x2 = sqrt(x1 * x1 + y1 * y1) * cos(PI + atan(y1 / x1) - angle);
                y2 = sqrt(x1 * x1 + y1 * y1) * sin(PI + atan(y1 / x1) - angle);
            }
        }
        else{
            if(y1 > 0){
                x2 = abs(y1) * cos(PI/2 - angle);
                y2 = abs(y1) * sin(PI/2 - angle);
            }
            else if(y1 < 0){
                x2 = abs(y1) * cos(3*PI/2 - angle);
                y2 = abs(y1) * sin(3*PI/2 - angle);
            }
            else{
                x2 = 0;
                y2 = 0;
            }
        }

        imput[0] = x2;
        imput[1] = -1 * y2;

        return imput;
    }

    public double[] imuDrive(double[]output, double[] imput, double angle, boolean stabilize,boolean fieldCentric){

        if(fieldCentric){
            compensate(imput,angle);
        }
        if(stabilize){
            stabilize(imput,angle);
        }

        //adds the raw values
        output[0] = (imput[1] - imput[0] + imput[2]);//leftFront
        output[1] = (imput[1] + imput[0] + imput[2]);//leftBack
        output[2] = (imput[1] + imput[0] - imput[2]);//rightFront
        output[3] = (imput[1] - imput[0] - imput[2]);//rightBack

        return output;
    }

    public double[] getPosition(double[] outputPosition, BNO055IMU imu1, BNO055IMU imu2,boolean restrict){
        double[] imputPosition = new double[3];

        //gets the raw values.
        imputPosition[0] = (imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle)/2;//Yaw
        imputPosition[1] = (imu1.getAngularOrientation().secondAngle + imu2.getAngularOrientation().secondAngle)/2;//Roll
        imputPosition[2] = (imu1.getAngularOrientation().thirdAngle + imu2.getAngularOrientation().thirdAngle)/2;//Pitch

        //The angles are on the circle can't just be averaged, so if the difference, is greater than pi, then it changes the value so it isn't
        for (int axis = 2; axis > 0 - 1; axis--) {
            if (imputPosition[axis] - outputPosition[axis] > PI) imputPosition[axis] -= 2 * PI;
            if (outputPosition[axis] - imputPosition[axis] > PI) imputPosition[axis] += 2 * PI;
        }

        smooth(outputPosition, imputPosition, .8, restrict);

        return outputPosition;
    }

    //initializes paramateres
    public void createIMU(BNO055IMU imu1,BNO055IMU imu2){
        //creates the imu and its parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu1.initialize(parameters);
        imu2.initialize(parameters);
    }
    public void calibrateIMU(BNO055IMU imu1, BNO055IMU imu2){
        // Get the calibration data and reset it
        BNO055IMU.CalibrationData calibrationData1 = imu1.readCalibrationData();
        File file1 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration1.json");
        ReadWriteFile.writeFile(file1, calibrationData1.serialize());

        //Creates the second imu
        BNO055IMU.CalibrationData calibrationData2 = imu2.readCalibrationData();
        File file2 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration2.json");
        ReadWriteFile.writeFile(file2, calibrationData2.serialize());
    }

    private double desiredAngle;
    private double rotation;
    private double currentAngle;

    public void startIMUturn(double turnAngle,BNO055IMU imu1,BNO055IMU imu2){
        double[] position = new double[3];
        getPosition(position,imu1,imu2,false);

        this.desiredAngle = turnAngle + position[0];
        PIDControl.startPID();
    }
    public double[] IMUturn(double[] motors,BNO055IMU imu1,BNO055IMU imu2){
        double[] imputs = new double[3];
        double[] position = new double[3];

        getPosition(position,imu1,imu2,false);
        this.currentAngle = position[0];

        PIDControl.onSensorChanged(position[0]);
        imputs[2] = PIDControl.getValue(2.9,1.6,.9,-.6);
        this.rotation = imputs[2];

        imuDrive(motors,imputs,0,false,false);
        return motors;
    }
    public boolean IMUturnCondidtion(double precision){
        boolean motorCondition = false;
        boolean angleCondition = false;
        boolean condidion = false;

        if(abs(rotation) < 10 * precision)motorCondition = true;
        else motorCondition = false;
        if(abs(currentAngle) < precision)angleCondition = true;
        else angleCondition = false;

        if(motorCondition && angleCondition) condidion = true;
        else condidion = false;

        return !condidion;
    }

    public double[]stabilize(double[] imput, double angle){
        this.desiredAngle += imput[2]/2;

        PIDControl.startPID();
        PIDControl.onSensorChanged(angle);

        imput[2] = PIDControl.getValue(2.9,1.6,.9,-.6);

        return imput;
    }


}