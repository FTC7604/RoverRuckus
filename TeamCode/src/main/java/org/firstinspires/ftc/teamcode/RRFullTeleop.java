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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

@TeleOp(name="RR Full Teleop", group="Linear Opmode")
//@Disabled


public class RRFullTeleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack    = null;
    private DcMotor rightBack   = null;

    private DcMotor leftLift = null;//looking dead on at the front of the bot
    private DcMotor rightLift = null;

    private DcMotor intake = null;
    private DcMotor intakeLift = null;

    private Servo hook = null;
    private Servo phoneMount = null;
    private Servo sampleArm = null;
    private Servo leftOutput = null;
    private Servo rightOutput = null;

    //hook
    private boolean hookCurrState = false;
    private boolean hookPrevState = false;
    private boolean hookIsOpen = true;

    //intake lift toggle controls
    private boolean intakeTargetIsUp = true;
    private boolean intakeCurrState = false;
    private boolean intakePrevState = false;



    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront  = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        leftLift = hardwareMap.get(DcMotor.class, "ll");
        rightLift = hardwareMap.get(DcMotor.class, "rl");
        intake = hardwareMap.get(DcMotor.class, "in");
        intakeLift = hardwareMap.get(DcMotor.class, "il");

        hook = hardwareMap.get(Servo.class, "hk");
        phoneMount = hardwareMap.get(Servo.class, "ph");
        sampleArm = hardwareMap.get(Servo.class, "sa");
        leftOutput = hardwareMap.get(Servo.class, "lo");
        rightOutput = hardwareMap.get(Servo.class, "ro");

        setMotorBehaviors();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            intake.setPower(gamepad2.right_stick_y);

            RunIntakeLift();

            RunLift();

            RunDrive();

            HoldPhoneAndSample();

            RunScoringBucket();

            RunHook();

            telemetry.addData("intake lift", intakeLift.getCurrentPosition());
            telemetry.addData("Encoder Position", leftLift.getCurrentPosition());
            telemetry.update();
        }
    }

    //toggles between engaged and disengaged position
    private void togglePosition(Servo servo, double disengagedPosition, double engagedPosition, boolean button){
        //default state, when the button isn't pressed it remains its engaged.
        if(!button){
            servo.setPosition(disengagedPosition);
        }
        //when the button is pressed it goes into its engaged state
        else if(button){
            servo.setPosition(engagedPosition);
        }
    }

    private void RunIntakeLift () {
        double intakeliftUp = 1;//power level for going up
        double intakeliftDown = -0.4;//for going down

        int intakeUpperLimit = 0;//for the robot intake all the way in this is how the game starts
        int intakeLowerLimit = -2000        //if the lift is the above where is should be
;//for the robot intake all the way extended, when picking up particles.


        //intake toggle controls to change where we want to go
        intakeCurrState = gamepad2.x;

        if ((gamepad2.x) && (intakeCurrState != intakePrevState)){
            intakeTargetIsUp = !intakeTargetIsUp;
        }
        intakePrevState = intakeCurrState;

        if ((intakeTargetIsUp) && (intakeLift.getCurrentPosition() < intakeUpperLimit))
        {
            intakeLift.setPower(intakeliftUp);
        }
        else if ((intakeTargetIsUp == false) && (intakeLift.getCurrentPosition() > intakeLowerLimit))
        {
            intakeLift.setPower(intakeliftDown);
        }
        else intakeLift.setPower(0);
    }

    private void RunLift () {
        int liftUpperLimit = (4030 - 100);//I just did the math for the values because android studio got mad
        int liftLowerLimit = (0 + 200);//the lift is all the way down, the plus is to compensate for lag.

        //Lifter controls, Negative is up, positive is down Hang is the other way around

        //if the lift is below where it is supposed to be
        if (leftLift.getCurrentPosition() < liftLowerLimit) {
            //you can go up
            if (gamepad2.left_stick_y > 0) {
                leftLift.setPower(gamepad2.left_stick_y);
                rightLift.setPower(gamepad2.left_stick_y);
            }
            //but not down
            else {
                leftLift.setPower(0);
                rightLift.setPower(0);
            }
        }
        //if the lift is the above where is should be
        else if (leftLift.getCurrentPosition() > liftUpperLimit) {
            //you can go down
            if (gamepad2.left_stick_y < 0) {
                leftLift.setPower(gamepad2.left_stick_y);
                rightLift.setPower(gamepad2.left_stick_y);
            }
            //but not down
            else
            {
                leftLift.setPower(0);
                rightLift.setPower(0);
            }
        }
        else{
            leftLift.setPower(gamepad2.left_stick_y);
            rightLift.setPower(gamepad2.left_stick_y);
        }
    }

    private void RunDrive () {
        double y = (((-gamepad1.left_stick_y)*(abs(-gamepad1.left_stick_y))+((-gamepad1.right_stick_y)*(abs(-gamepad1.right_stick_y))))/2);
        double x = (((-gamepad1.left_stick_x)*(abs(-gamepad1.left_stick_x))+((-gamepad1.right_stick_x)*(abs(-gamepad1.right_stick_x))))/2);
        double turnVal = (((-gamepad1.left_stick_y)-(-gamepad1.right_stick_y))/2);

        leftFront.setPower(y-x+turnVal);
        leftBack.setPower(y+x+turnVal);
        rightFront.setPower(y+x-turnVal);
        rightBack.setPower(y-x-turnVal);

        telemetry.addData("y", y);
        telemetry.addData("x", x);
        telemetry.addData("turnval", turnVal);
    }

    private void HoldPhoneAndSample () {
        //phone mount
        double phoneIn = 0.23;
        double phoneOut = 0.75;

        //sample arm
        double armDown = 1;
        double armUp = 0.1;

        //hold the phone and arm in place
        phoneMount.setPosition(phoneIn);
        sampleArm.setPosition(armUp);
    }

    private void RunScoringBucket () {
        //ScoringBucket
        double loDown = 0;
        double loUp = 0.75;
        double roDown = 1- loDown;
        double roUp = 1-loUp;

        double liftHalfway = 2500;

        //controls of the bucket with gamapad1.a and makes sure that the user can't break the bot. It holds at a diagonal position when it can so that it keeps the particles
        if ((gamepad2.a == false) && leftLift.getCurrentPosition() > liftHalfway){
            leftOutput.setPosition(0.35);
            rightOutput.setPosition(0.65);
        }
        else if (leftLift.getCurrentPosition() < liftHalfway){
            togglePosition(leftOutput, loDown, (loDown+0.1), gamepad2.a);//lets the user move the bucket around but only a little bit
            togglePosition(rightOutput, roDown, (roDown-0.1), gamepad2.a);
        }
        else{
            togglePosition(leftOutput, loDown, loUp, gamepad2.a);//full range of motion for scoring when the bucket is in the air
            togglePosition(rightOutput, roDown, roUp, gamepad2.a);
        }
    }

    private void RunHook () {
        double hookOpen = 0.2;
        double hookEngaged = 0.6;

        //this block of code controls the position of the hook with a toggle switch.
        hookCurrState = gamepad2.b;
        //toggle the hook position with gamepad2.b
        if ((gamepad2.b) && hookCurrState != hookPrevState){
            hookIsOpen = !hookIsOpen;//change the hook target
        }
        hookPrevState = hookCurrState;

        if (hookIsOpen) hook.setPosition(hookOpen);
        else hook.setPosition(hookEngaged);

    }

    void setMotorBehaviors () {
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        intakeLift.setDirection(DcMotor.Direction.FORWARD);

        intakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //makes it stop when the motor is at rest
        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
