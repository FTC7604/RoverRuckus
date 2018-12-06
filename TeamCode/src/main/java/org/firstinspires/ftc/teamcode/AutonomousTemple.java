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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//starts the autonomous
/*@Autonomous(name="Deploy Test", group="Linear Opmode")
//@Disabled
public class AutonomousTemple extends LinearOpMode {
    //creates the runtime
    private ElapsedTime runtime = new ElapsedTime();

    //drivetrain
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack    = null;
    private DcMotor rightBack   = null;

    //lift motors
    private DcMotor leftLift = null;//looking dead on at the front of the bot
    private DcMotor rightLift = null;

    //intake motors
    private DcMotor intake = null;
    private DcMotor intakeLift = null;

    //all of the servos
    private Servo hook = null;
    private Servo phoneMount = null;
    private Servo sampleArm = null;
    private Servo leftOutput = null;
    private Servo rightOutput = null;

    //hook
    private static final double hookOpen = 0.2;
    private static final double hookEngaged = 0.6;

    //ScoringBucket
    private static final double loDown = 0;
    private static final double loUp = 0.75;
    private static final double roDown = 1- loDown;
    private static final double roUp = 1-loUp;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //drivetrain
        leftFront  = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        //output and intake
        leftLift = hardwareMap.get(DcMotor.class, "ll");
        rightLift = hardwareMap.get(DcMotor.class, "rl");
        intake = hardwareMap.get(DcMotor.class, "in");
        intakeLift = hardwareMap.get(DcMotor.class, "il");

        //servos
        hook = hardwareMap.get(Servo.class, "hk");
        phoneMount = hardwareMap.get(Servo.class, "ph");
        sampleArm = hardwareMap.get(Servo.class, "sa");
        leftOutput = hardwareMap.get(Servo.class, "lo");
        rightOutput = hardwareMap.get(Servo.class, "ro");

        //sets the direction for the lift
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        //sets the direction for the intake
        intake.setDirection(DcMotor.Direction.FORWARD);
        intakeLift.setDirection(DcMotor.Direction.FORWARD);

        //resets all the encoders
        intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //starts all the encoders
        intakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //makes it stop when the motor is at rest
        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //sets the directioon for the drivetrain
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //when its 0 brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        runtime.reset();

        Deploy();


    }

    void Deploy() {
        int liftUpperLimit = (3530 - 100);//I just did the math for the values because android studio got mad
        int liftLowerLimit = (0 + 200);//the lift is all the way down, the plus is to compensate for lag.

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setTargetPosition(liftUpperLimit);

        //lower down then stop
        while (leftLift.isBusy()){
            leftLift.setPower(1);
            rightLift.setPower(1);
        }
        leftLift.setPower(0);
        rightLift.setPower(0);
    }

//    void Sample(int mineralPosition){
//        switch(mineralPosition){
//            case 1: go back
//        }
//    }
}*/