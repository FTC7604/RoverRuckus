package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import static java.lang.Math.*;

public class IMUControl {
    //will convert an angle into a value that is greater than or equal to 0 and less than 2pi
    public double calcAngle(double angle) {
        //a is the angle
        //greater than or equal to 0
        while (angle < 0) {
            angle += 2 * PI;
        }
        //less than pi
        while (angle >= 2 * PI) {
            angle -= 2 * PI;
        }
        return angle;
    }

    //smooths data
    public void smooth(double[] smoothData, double[] newData, double fraction) {

        for (int axis = 2; axis > -1; axis--) {
            //smooths the data
            smoothData[axis] = ((1 - fraction) * smoothData[axis]) + (fraction * newData[axis]);
            //prevents the data from remaining within the bounds
            smoothData[0] = calcAngle(smoothData[0]);
        }

    }

    public double[] compensate(double[] imput, double angle) {
        //0 is the x values
        //1 is the y values

        double[] output = new double[2];

        if (imput[0] != 0) {
            output[0] = sqrt(pow(imput[0], 2) + pow(imput[1], 2)) * (cos(angle - atan(imput[1] / imput[0])));
            output[1] = sqrt(pow(imput[0], 2) + pow(imput[1], 2)) * (sin(angle - atan(imput[2] / imput[0])));
        } else {
            if (imput[1] > 0) {
                output[0] = imput[1] * (cos(angle - PI / 2));
                output[1] = imput[1] * (sin(angle - PI / 2));
            } else if (imput[2] < 0) {
                output[0] = imput[1] * (cos(angle - 3 * PI / 2));
                output[1] = imput[1] * (sin(angle - 3 * PI / 2));
            } else {
                output[0] = 0;
                output[1] = 0;
            }
        }
        if (output[0] > 0) {
            output[0] *= -1;
        }

        return output;
    }

    public double[] imuDrive(double[]output, double[] imput, double[] position, boolean c){

        if(c) {
            compensate(imput,position[0]);
        }

        //adds the raw values
        output[0] = (imput[1] - imput[0] + imput[2]);//leftFront
        output[1] = (imput[1] + imput[0] + imput[2]);//leftBack
        output[2] = (imput[1] + imput[0] - imput[2]);//rightFront
        output[3] = (imput[1] - imput[0] - imput[2]);//rightBack

        return output;
    }

    //Conceptually I don't understand how to write this method
//    private void imuTurn(double angle, double position){
//        double netAngle = angle - position;
//        double variablility = 0.2;
//
//        while(netAngle < newPosition[0] - variablility && netAngle > newPosition[0] + variablility) {
//            imuDrive(0, 0, netAngle - newPosition[0], false);
//        }
//    }

    public double[] getPosition(double[] outputPosition, BNO055IMU imu1, BNO055IMU imu2){
        double[] imputPosition = new double[3];

        //gets the raw values.
        imputPosition[0] = (imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle)/2;//Yaw
        imputPosition[1] = (imu1.getAngularOrientation().secondAngle + imu2.getAngularOrientation().secondAngle)/2;//Roll
        imputPosition[2] = (imu1.getAngularOrientation().thirdAngle + imu2.getAngularOrientation().thirdAngle)/2;//Pitch

        //The angles are on the circle can't just be averaged, so if the difference, is greater than pi, then it changes the value so it isn't
        for(int axis = 2; axis > 0-1; axis--){
            if (imputPosition[axis] - outputPosition[axis] > PI) imputPosition[axis] -= 2 * PI;
            if (outputPosition[axis] - imputPosition[axis] > PI) imputPosition[axis] += 2 * PI;
        }

        smooth(outputPosition, imputPosition, .4);

        return outputPosition;
    }

    //initializes paramateres
    public void startIMU(BNO055IMU imu1,BNO055IMU imu2){
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
        String filename1 = "AdafruitIMUCalibration1.json";
        File file1 = AppUtil.getInstance().getSettingsFile(filename1);
        ReadWriteFile.writeFile(file1, calibrationData1.serialize());

        //Creates the second imu
        BNO055IMU.CalibrationData calibrationData2 = imu2.readCalibrationData();
        String filename2 = "AdafruitIMUCalibration2.json";
        File file2 = AppUtil.getInstance().getSettingsFile(filename2);
        ReadWriteFile.writeFile(file2, calibrationData2.serialize());
    }



}

