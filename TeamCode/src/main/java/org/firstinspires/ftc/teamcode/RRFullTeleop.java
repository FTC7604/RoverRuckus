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

import org.firstinspires.ftc.teamcode.util.Crunchy;
import org.firstinspires.ftc.teamcode.util.IMUControl;
import org.firstinspires.ftc.teamcode.util.PropertiesLoader;
import org.firstinspires.ftc.teamcode.util.vision.VisionTarget;
import org.firstinspires.ftc.teamcode.util.vision.VisionTracking;

import java.util.Locale;

import static java.lang.Math.abs;
import static java.lang.Math.ceil;
import static java.lang.Math.pow;
import static java.lang.Math.signum;

@TeleOp(name="RR Full Teleop", group="Linear Opmode")
//@Disabled

public class RRFullTeleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private org.firstinspires.ftc.teamcode.util.IMUControl IMUControl = new IMUControl();
    //hook
    private boolean hookCurrState = false;
    private boolean hookPrevState = false;
    private boolean hookIsOpen = true;

    //intake lift toggle controls
    private boolean intakeTargetIsUp = true;
    private boolean intakeCurrState = false;
    private boolean intakePrevState = false;

    private Crunchy crunchy;

    private PropertiesLoader loader = new PropertiesLoader("RRFullTeleop");
    private final double SLOW_MULTIPLIER = loader.getDoubleProperty("slowMultiplier");

    @Override
    public void runOpMode() {
        crunchy = new Crunchy(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        setMotorBehaviors();
        
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            crunchy.intake.setPower(gamepad2.right_stick_y);
//            runIntake();
            
            RunIntakeLift();

            RunLift();

            RunDrive();

            HoldPhoneAndSample();

            RunScoringBucket();

            RunHook();

            telemetry.addData("intake lift", crunchy.intakeLift.getCurrentPosition());
            telemetry.addData("Encoder Position", crunchy.liftLeft.getCurrentPosition());
            telemetry.update();
        }
    }
    
    private void runIntake(){
        int intakePosition = crunchy.intake.getCurrentPosition() % (1440 / 2);
        if(abs(gamepad2.right_stick_y) > 2) {
            crunchy.intake.setPower(gamepad2.right_stick_y);
        }
        else if(intakePosition > (1440 - 200)/2 || intakePosition < 100) {
                crunchy.intake.setPower(.5);
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
        int intakeLowerLimit = -1500;       //if the lift is the above where is should be
//for the robot intake all the way extended, when picking up particles.


        //intake toggle controls to change where we want to go
        intakeCurrState = gamepad2.x;

        if ((gamepad2.x) && (intakeCurrState != intakePrevState)){
            intakeTargetIsUp = !intakeTargetIsUp;
        }
        intakePrevState = intakeCurrState;

        if ((intakeTargetIsUp) && (crunchy.intakeLift.getCurrentPosition() < intakeUpperLimit))
        {
            crunchy.intakeLift.setPower(intakeliftUp);
        }
        else if ((!intakeTargetIsUp) && (crunchy.intakeLift.getCurrentPosition() > intakeLowerLimit))
        {
            crunchy.intakeLift.setPower(intakeliftDown);
        }
        else crunchy.intakeLift.setPower(0);
    }

    private void RunLift () {
        int liftUpperLimit = (3889);//I just did the math for the values because android studio got mad
        int liftLowerLimit = (0 + 75);//the lift is all the way down, the plus is to compensate for lag.

        //Lifter controls, Negative is up, positive is down Hang is the other way around

        //if the lift is below where it is supposed to be
        if (crunchy.liftLeft.getCurrentPosition() < liftLowerLimit) {
            //you can go up
            if (gamepad2.left_stick_y > 0) {
                crunchy.liftLeft.setPower(gamepad2.left_stick_y);
                crunchy.liftRight.setPower(gamepad2.left_stick_y);
            }
            //but not down
            else {
                crunchy.liftLeft.setPower(0);
                crunchy.liftRight.setPower(0);
            }
        }
        //if the lift is the above where is should be
        else if (crunchy.liftLeft.getCurrentPosition() > liftUpperLimit) {
            //you can go down
            if (gamepad2.left_stick_y < 0) {
                crunchy.liftLeft.setPower(gamepad2.left_stick_y);
                crunchy.liftRight.setPower(gamepad2.left_stick_y);
            }
            //but not down
            else
            {
                crunchy.liftLeft.setPower(0);
                crunchy.liftRight.setPower(0);
            }
        }
        else{
            crunchy.liftLeft.setPower(gamepad2.left_stick_y);
            crunchy.liftRight.setPower(gamepad2.left_stick_y);
        }
    }

    private boolean currentDriveMode = true;
    private boolean pastDriveMode = true;
    private boolean fieldCentric = false;

    private void RunDrive () {
        double[]controller = new double[3];
        double[]motors = new double[4];
        double[]position = new double[3];

        currentDriveMode = gamepad1.left_bumper;
        if(currentDriveMode && !pastDriveMode){
            if(fieldCentric)fieldCentric = false;
            if(!fieldCentric)fieldCentric = true;
        }
        pastDriveMode = currentDriveMode;

        if(!fieldCentric){
            controller[1] = (((-gamepad1.left_stick_y) * (abs(-gamepad1.left_stick_y)) + ((-gamepad1.right_stick_y) * (abs(-gamepad1.right_stick_y)))) / 2);
            controller[0] = (((-gamepad1.left_stick_x) * (abs(-gamepad1.left_stick_x)) + ((-gamepad1.right_stick_x) * (abs(-gamepad1.right_stick_x)))) / 2);
            controller[2] = (((-gamepad1.left_stick_y) - (-gamepad1.right_stick_y)) / 2);
        }
        else {
            controller[0] = pow(-gamepad1.left_stick_x, 3); //desired x movement
            controller[1] = pow(-gamepad1.left_stick_y, 3); //desired y movement
            controller[2] = pow(gamepad1.right_stick_x, 3); //desired rotation
        }

        if(gamepad1.right_bumper) {
            controller[0] *= SLOW_MULTIPLIER;
            controller[1] *= SLOW_MULTIPLIER;
            controller[2] *= abs(SLOW_MULTIPLIER);
        }

        if(fieldCentric) {
            IMUControl.getPosition(position, crunchy.imu1, crunchy.imu2, true);
            IMUControl.imuDrive(motors, controller, position[0], false, true);
        }
        else{
            IMUControl.imuDrive(motors, controller, position[0], false, false);
        }

        crunchy.frontLeft.setPower(motors[0]);
        crunchy.backLeft.setPower(motors[1]);
        crunchy.frontRight.setPower(motors[2]);
        crunchy.backRight.setPower(motors[3]);

        telemetry.addData("controller_x", controller[0]);
        telemetry.addData("controller_y", controller[1]);
        telemetry.addData("controller_r", controller[2]);
    }

    private void HoldPhoneAndSample () {
        //phone mount
        double phoneIn = 0.23;
        double phoneOut = 0.75;

        //sample arm
        double armDown = 1;
        double armUp = 0.1;

        //hold the phone and arm in place
        crunchy.phoneMount.setPosition(phoneIn);
        crunchy.sampleArm.setPosition(armUp);
    }

    private void RunScoringBucket () {
        //ScoringBucket
        double loDown = 0;
        double loUp = 0.75;
        double roDown = 1- loDown;
        double roUp = 1-loUp;

        double liftHalfway = 3100;

        //controls of the bucket with gamapad1.a and makes sure that the user can't break the bot. It holds at a diagonal position when it can so that it keeps the particles
        if ((gamepad2.a == false) && crunchy.liftLeft.getCurrentPosition() > liftHalfway){
            crunchy.leftOutput.setPosition(0.35);
            crunchy.rightOutput.setPosition(0.65);
        }
        else if (crunchy.liftLeft.getCurrentPosition() < liftHalfway){
            togglePosition(crunchy.leftOutput, loDown, (loDown+0.1), gamepad2.a);//lets the user move the bucket around but only a little bit
            togglePosition(crunchy.rightOutput, roDown, (roDown-0.1), gamepad2.a);
        }
        else{
            togglePosition(crunchy.leftOutput, loDown, loUp, gamepad2.a);//full range of motion for scoring when the bucket is in the air
            togglePosition(crunchy.rightOutput, roDown, roUp, gamepad2.a);
        }
    }

    private void RunHook () {
        double hookOpen = 0.2;
        double hookEngaged = 0.6;

        //this block of code controls the position of the hook with a toggle switch.
        hookCurrState = gamepad2.y;
        //toggle the hook position with gamepad2.y
        if ((gamepad2.y) && hookCurrState != hookPrevState){
            hookIsOpen = !hookIsOpen;//change the hook target
        }
        hookPrevState = hookCurrState;

        if (hookIsOpen) crunchy.hook.setPosition(hookOpen);
        else crunchy.hook.setPosition(hookEngaged);

    }

    private void setMotorBehaviors () {
        crunchy.liftLeft.setDirection(DcMotor.Direction.FORWARD);
        crunchy.liftRight.setDirection(DcMotor.Direction.REVERSE);

        crunchy.intake.setDirection(DcMotor.Direction.FORWARD);
        crunchy.intakeLift.setDirection(DcMotor.Direction.FORWARD);

        //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        crunchy.intakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        crunchy.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        crunchy.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //makes it stopAndResetEncoders when the motor is at rest
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        crunchy.intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        crunchy.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        crunchy.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        crunchy.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        crunchy.frontRight.setDirection(DcMotor.Direction.FORWARD);
        crunchy.backLeft.setDirection(DcMotor.Direction.REVERSE);
        crunchy.backRight.setDirection(DcMotor.Direction.FORWARD);

        crunchy.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        crunchy.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        crunchy.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        crunchy.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
