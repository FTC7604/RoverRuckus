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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Math.*;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.IMUControl;

@TeleOp(name="Mechanum 5.0", group="Linear Opmode")
@Disabled
public class IMUmecDrive extends LinearOpMode {

    private org.firstinspires.ftc.teamcode.util.IMUControl IMUControl = new IMUControl();

    private ElapsedTime runtime = new ElapsedTime();

    //creates the 4 motors
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack    = null;
    private DcMotorEx rightBack   = null;

    //creates the imus
    private BNO055IMU imu1 = null;
    private BNO055IMU imu2 = null;

    private void setUP(){
        createHardwareMap();
        setDirections();
        setEncoders();
        setBrakes();
    }

    private void createHardwareMap(){
        //drivetrain
        leftFront  = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        rightFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        leftBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        rightBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");

//        //output and intake
//        leftLift = hardwareMap.get(DcMotor.class, "ll");
//        rightLift = hardwareMap.get(DcMotor.class, "rl");
//        intake = hardwareMap.get(DcMotor.class, "in");
//        intakeLift = hardwareMap.get(DcMotor.class, "il");
//
//        //servos
//        hook = hardwareMap.get(Servo.class, "hk");
//        phoneMount = hardwareMap.get(Servo.class, "ph");
//        sampleArm = hardwareMap.get(Servo.class, "sa");
//        leftOutput = hardwareMap.get(Servo.class, "lo");
//        rightOutput = hardwareMap.get(Servo.class, "ro");

        //imus
        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
    }
    private void setDirections(){
//        //sets the direction for the lift
//        leftLift.setDirection(DcMotor.Direction.FORWARD);
//        rightLift.setDirection(DcMotor.Direction.REVERSE);
//
//        //sets the direction for the intake
//        intake.setDirection(DcMotor.Direction.FORWARD);
//        intakeLift.setDirection(DcMotor.Direction.FORWARD);

        //sets the directioon for the drivetrain
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
    }
    private void setEncoders(){
//        //resets the aux encoders
//        intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //resets the drive encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        //starts the aux encoders
//        intakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //resets the drive encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void setBrakes(){
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //starts the Opmode
    @Override
    public void runOpMode() {
        //says that its good to go
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //sets everything up, motors and encoders
        setUP();

        //waits for the start
        waitForStart();
        runtime.reset();

        //Sets up the IMU
        IMUControl.createIMU(imu1,imu2);
        IMUControl.calibrateIMU(imu1,imu2);

        double[]controller = new double[3];
        double[]motors = new double[4];
        double[]position = new double[3];

        //loop that starts the opmode
        while (opModeIsActive()) {
            controller[0] = pow(-gamepad1.left_stick_x,3); //desired x movement
            controller[1] = pow(-gamepad1.left_stick_y,3); //desired y movement
            controller[2] = pow(gamepad1.right_stick_x,3); //desired rotation

            telemetry.addData("start_controller_x", controller[0]);
            telemetry.addData("start_controller_y", controller[1]);

            IMUControl.getPosition(position,imu1,imu2,true);
            IMUControl.imuDrive(motors,controller,position[0],true,false);

            imputMecMotors(motors);

            loopCounter++;
            telemetry.addData("Loops per second", ((int)(loopCounter/time)));
            telemetry.addData("rotation position", position[0]);
            telemetry.addData("end_controller_x", controller[0]);
            telemetry.addData("end_controller_y", controller[1]);
            telemetry.update();
        }



    }
    private int loopCounter;

    private void imputMecMotors(double[]imputs){
        leftFront.setPower(imputs[0]);
        leftBack.setPower(imputs[1]);
        rightFront.setPower(imputs[2]);
        rightBack.setPower(imputs[3]);
    }
    private void imputMecVelocity(double[]imputs){
        arrayScale(imputs,.0175 * 39600);
        leftFront.setVelocity(imputs[0], AngleUnit.DEGREES);
        leftBack.setVelocity(imputs[1], AngleUnit.DEGREES);
        rightFront.setVelocity(imputs[2], AngleUnit.DEGREES);
        rightBack.setVelocity(imputs[3], AngleUnit.DEGREES);
    }


    private double[] arrayScale(double[] imput, double scalar){
        for(int i = imput.length; i > 0; i--){
            imput[i-1] *= scalar;
        }
        return imput;
    }
}
