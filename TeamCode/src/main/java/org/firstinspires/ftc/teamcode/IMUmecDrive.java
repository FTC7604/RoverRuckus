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

@TeleOp(name="Mechanum 5.0", group="Linear Opmode")
//@Disabled
public class IMUmecDrive extends LinearOpMode {

    private IMUControl IMUControl = new IMUControl();

    private ElapsedTime runtime = new ElapsedTime();

    //creates the 4 motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack    = null;
    private DcMotor rightBack   = null;

    //creates the imus
    private BNO055IMU imu1 = null;
    private BNO055IMU imu2 = null;

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
        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");

        //reverses the left motors, so that they can be programed the same
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

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
            IMUControl.imuDrive(motors,controller,position[0],false,true);

            imputMecMotors(motors);

            loopCounter++;
            telemetry.addData("Loops per second", ((int)(loopCounter/time)));
            telemetry.addData("rotation position", position[0]);
            telemetry.addData("end_controller_x", controller[0]);
            telemetry.addData("end_controller_y", controller[1]);
            telemetry.update();
        }



    }

    private void imputMecMotors(double[]imputs){
        leftFront.setPower(imputs[0]);
        leftBack.setPower(imputs[1]);
        rightFront.setPower(imputs[2]);
        rightBack.setPower(imputs[3]);
    }

    private int loopCounter;
}
