/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//Ignore this
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Math.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

@TeleOp(name="Mechanum 4.0", group="Linear Opmode")
//@Disabled
public class IMUmecDrive extends LinearOpMode {

    //creates the runtime
    private ElapsedTime runtime = new ElapsedTime();

    //creates the 4 motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack    = null;
    private DcMotor rightBack   = null;

    //starts the Opmode
    @Override
    public void runOpMode() {
        //says that its good to go
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //creates the hardwaremap
        leftFront  = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        //the imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //7. 2 imus
//        imu = hardwareMap.get(BNO055IMU.class, "li");
//        imu = hardwareMap.get(BNO055IMU.class, "ri");

        //reverses the left motors, so that they can be programed the same
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //waits for the start
        waitForStart();
        runtime.reset();

        //3. Sets up the IMU
        startIMU();
        calibrateIMU();

        // 4. A though for gyro PID
        // double desiredPosition = 0;
        // double rotationPower = 0;

        //loop that starts the opmode
        while (opModeIsActive()) {

            imuDrive();

            loopCounter++;
            telemetry.addData("Loops per second", ((int)(loopCounter/time)));
            //3. Print out the values to ensure that they make sense
            //telemetry.addData("smooth_position", smoothPosition[0]);
            //telemetry.addData("new_position", newPosition[0]);
            telemetry.update();
        }

    }
    private int loopCounter;

    //8. Run all the imu code as one method.
    private void imuDrive(){

        //imputs from controller
        double controller_y = pow(gamepad1.left_stick_y,3);
        double controller_x = pow(-gamepad1.left_stick_x,3);
        double controller_rotation = pow(gamepad1.right_stick_x,3);
        //4. A thought for gryo PID
        // desiredPosition -= gamepad1.right_stick_x;
        // rotationPower = desiredPosition - smoothPosition[0];


        //3.Sets the array for the IMU, then changes the rotation value
        getPosition(newPosition);
        smooth(smoothPosition,newPosition,.1);

        //1. Imput Motor function
        getRobot_movement(controller_x,controller_y);
        getRobot_rotation(controller_rotation);
        ////5. Gyro PID
        //getRobot_rotation(rotationPower);
        getMotor_imputs();
        ////5. Gyro PID
        //getMotor_imputs(robot_movement,rotationPower);
        imputMecanumMotors();

        //updates the telemetry
        telemetry.addData("controller_y", controller_y);
        telemetry.addData("controller_x", controller_x);
        telemetry.addData("controller_rotation", controller_rotation);
//            telemetry.addData("bot_rotation_1", robot_rotation[0]);
//            telemetry.addData("bot_rotation_2", robot_rotation[2]);
//            telemetry.addData("bot_imput_1", motor_imputs[0]);
//            telemetry.addData("bot_imput_2", motor_imputs[2]);
    }

    //will convert an angle into a value that is greater than or equal to 0 and less than 2pi
    double calcAngle(double angle){
        //a is the angle
        //greater than or equal to 0
        while(angle < 0){
            angle += 2 * PI;
        }
        //less than pi
        while(angle >= 2 * PI){
            angle -= 2 * PI;
        }
        return angle;
    }

    //smooths data
    double[] smooth(double[] smoothData, double[] newData, double fraction){

        for(int axis = 2; axis > -1; axis--){
            //smooths the data
            smoothData[axis] = ((1-fraction) * smoothData[axis]) + (fraction * newData[axis]);
            //prevents the data from remaining within the bounds
            smoothData[0] = calcAngle(smoothData[0]);
        }

        return smoothData;
    }

    //1. Creates class arrays that can be altered induvidually.
    private double [] robot_movement = new double[4];
    private double [] robot_rotation = new double[4];
    private double [] motor_imputs = new double[4];
    //0 is left front
    //1 is left back
    //2 is right front
    //3 is right back

    private void getRobot_movement(double x1, double y1){
        double x2 = 0;
        double y2 = 0;

//        //6. In order for the code to work, the angles must be worked out properly.
//        if(x1 * y1 > 0) {
//            x2 = sqrt(pow(x1, 2) + pow(y1, 2)) * (cos(smoothPosition[0] - atan(y1 / x1)));
//        }
//        else {
//            x2 = sqrt(pow(x1, 2) + pow(y1, 2)) * (2 * PI - cos(smoothPosition[0] - atan(y1 / x1)));
//        }
        if(x1 != 0) {
            x2 = sqrt(pow(x1, 2) + pow(y1, 2)) * (cos(smoothPosition[0] - atan(y1 / x1)));
            y2 = sqrt(pow(x1, 2) + pow(y1, 2)) * (sin(smoothPosition[0] - atan(y1 / x1)));
        }
        else{
            if(y1 > 0){
                x2 = sqrt(pow(x1, 2) + pow(y1, 2)) * (cos(smoothPosition[0] - PI/2));
                y2 = sqrt(pow(x1, 2) + pow(y1, 2)) * (sin(smoothPosition[0] - PI/2));
            }
            else if(y1 < 0){
                x2 = sqrt(pow(x1, 2) + pow(y1, 2)) * (cos(smoothPosition[0] - 3*PI/2));
                y2 = sqrt(pow(x1, 2) + pow(y1, 2)) * (sin(smoothPosition[0] - 3*PI/2));
            }
            else{
                x2 = 0;
                y2 = 0;
            }
        }

        //adds the raw values
        robot_movement[0] = y2 - x2;
        robot_movement[1] = y2 + x2;
        robot_movement[2] = y2 + x2;
        robot_movement[3] = y2 - x2;

        telemetry.addData("robot_x", x2);
        telemetry.addData("robot_y", y2);
    }

    private void getRobot_rotation(double r){
        robot_rotation[0] = r;
        robot_rotation[1] = r;
        robot_rotation[2] = -r;
        robot_rotation[3] = -r;

        telemetry.addData("robot_rotation", r);
    }

    private void getMotor_imputs(){

        motor_imputs[0] = robot_rotation[0] + robot_movement[0];
        motor_imputs[1] = robot_rotation[1] + robot_movement[1];
        motor_imputs[2] = robot_rotation[2] + robot_movement[2];
        motor_imputs[3] = robot_rotation[3] + robot_movement[3];
    }

    private void imputMecanumMotors(){
        leftFront.setPower(motor_imputs[0]);
        leftBack.setPower(motor_imputs[1]);
        rightFront.setPower(motor_imputs[2]);
        rightBack.setPower(motor_imputs[3]);
    }

    //3. Creates the IMU then its array
    private BNO055IMU imu = null;

    //7. 2 imus
//    private BNO055IMU left_imu = null;
//    private BNO055IMU right_imu = null;

    //initializes paramateres
    private void startIMU(){
        //creates the imu and its parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        //7. 2 imus
//        BNO055IMU.Parameters left_parameters = new BNO055IMU.Parameters();
//        left_parameters.loggingEnabled = true;
//        left_parameters.loggingTag = "left_imu";
//        left_imu.initialize(left_parameters);
//
//        BNO055IMU.Parameters right_parameters = new BNO055IMU.Parameters();
//        right_parameters.loggingEnabled = true;
//        right_parameters.loggingTag = "right_imu";
//        right_imu.initialize(right_parameters);
    }

    //resets those parameters
    private void calibrateIMU(){
        // Get the calibration data and reset it
        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());

        //7. 2 imus
//        BNO055IMU.CalibrationData left_calibration = left_imu.readCalibrationData();
//        String left_filename = "left_imu.json";
//        File left_file = AppUtil.getInstance().getSettingsFile(left_filename);
//        ReadWriteFile.writeFile(left_file, left_calibration.serialize());
//
//        BNO055IMU.CalibrationData right_calibration = right_imu.readCalibrationData();
//        String right_filename = "right_imu.json";
//        File right_file = AppUtil.getInstance().getSettingsFile(right_filename);
//        ReadWriteFile.writeFile(right_file, right_calibration.serialize());
    }

    private double[]smoothPosition = new double[3];
    private double[]newPosition = new double[3];

    //creates the new position for the new value
    private double[] getPosition(double[]newPosition){

        //gets the raw values.
        newPosition[0] = (imu.getAngularOrientation().firstAngle);//Yaw
        newPosition[1] = (imu.getAngularOrientation().secondAngle);//Roll
        newPosition[2] = (imu.getAngularOrientation().thirdAngle);//Pitch

        //7. 2 imus
//        newPosition[0] = (left_imu.getAngularOrientation().firstAngle + right_imu.getAngularOrientation().firstAngle)/2;
//        newPosition[1] = (left_imu.getAngularOrientation().secondAngle + right_imu.getAngularOrientation().secondAngle)/2;
//        newPosition[2] = (left_imu.getAngularOrientation().thirdAngle + right_imu.getAngularOrientation().thirdAngle)/2;

        //The angles are on the circle can't just be averaged, so if the difference, is greater than pi, then it changes the value so it isn't
        for(int axis = 2; axis > 0-1; axis--){
            if (newPosition[axis] - smoothPosition[axis] > PI) newPosition[axis] -= 2 * PI;
            if (smoothPosition[axis] - newPosition[axis] > PI) newPosition[axis] += 2 * PI;
        }

        return newPosition;
    }
}
