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


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Crunchy;
import org.firstinspires.ftc.teamcode.util.PropertiesLoader;

import java.util.Locale;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GOLD;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

@TeleOp(name="RR Full Teleop", group="Linear Opmode")
//@Disabled

public class RRFullTeleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //hook
    private boolean hookCurrState = false;
    private boolean hookPrevState = false;
    private boolean hookIsOpen = true;

    //intake lift toggle controls
    private boolean intakeTargetIsUp = false;
    private boolean intakeCurrState = false;
    private boolean intakePrevState = false;

    private Crunchy crunchy;

    private PropertiesLoader loader = new PropertiesLoader("RRFullTeleop");
    private final double SLOW_MULTIPLIER = loader.getDoubleProperty("slowMultiplier");
    private final double SLOW_SERVO_SPEED = loader.getDoubleProperty("slowServoSpeed");
    private final boolean SHOW_COLOR_FORMAT_DATA = loader.getBooleanProperty("showColorFormatData");
    private final int COLOR_SENSOR_LOOP_CYCLES = loader.getIntegerProperty("colorSensorLoopCycles");

    //stuff that I need to put in crunchy but am too excited to set up the leds
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Particle left;
    Particle right;

    //little enum for the color sensors.
    private enum Particle {
        NONE,
        YELLOW,
        WHITE,
    }

    private long startTime = 0;

    private int time()
    {
        return (int) ((System.currentTimeMillis() - startTime) / 1000);
    }

    int colorLoop = 0;

    @Override
    public void runOpMode() {
        crunchy = new Crunchy(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        setMotorBehaviors();

        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        crunchy.blinkinLedDriver.setPattern(pattern);

        waitForStart();
        runtime.reset();

        startTime = System.currentTimeMillis();

        while (opModeIsActive()) {

            crunchy.intake.setPower(gamepad2.right_stick_y);
//            runIntake();

            RunLift();

            RunIntakeLift();

            RunDrive();

            HoldPhoneAndSample();

            RunScoringBucket();

            RunHook();

//            telemetry.addData("intake lift", crunchy.intakeLift.getCurrentPosition());
//            telemetry.addData("Encoder Position", crunchy.liftLeft.getCurrentPosition());

            if (SHOW_COLOR_FORMAT_DATA) {
                final String colorFormat = "color=[%d,%d,%d], distance=%f mm";
                telemetry.addData("Color L", String.format(Locale.US, colorFormat,
                        crunchy.colorLeft.red(),
                        crunchy.colorLeft.green(),
                        crunchy.colorLeft.blue(),
                        crunchy.distanceLeft.getDistance(DistanceUnit.MM)));
                telemetry.addData("Color R", String.format(Locale.US, colorFormat,
                        crunchy.colorRight.red(),
                        crunchy.colorRight.green(),
                        crunchy.colorRight.blue(),
                        crunchy.distanceRight.getDistance(DistanceUnit.MM)));

            }


//            if (isYellow(crunchy.colorLeft.red(), crunchy.colorLeft.green(), crunchy.colorLeft.blue()))
//                left = Particle.YELLOW;
//            else if (isWhite(crunchy.colorLeft.red(), crunchy.colorLeft.green(), crunchy.colorLeft.blue()))
//                left = Particle.WHITE;
//            else left = Particle.NONE;
//
//            if (isYellow(crunchy.colorRight.red(), crunchy.colorRight.green(), crunchy.colorRight.blue()))
//                right = Particle.YELLOW;
//            else if (isWhite(crunchy.colorRight.red(), crunchy.colorRight.green(), crunchy.colorRight.blue()))
//                right = Particle.WHITE;
//            else right = Particle.NONE;

            if(colorLoop++ == COLOR_SENSOR_LOOP_CYCLES)
            {
                colorLoop = 0;
                RevBlinkinLedDriver.BlinkinPattern oldPattern = pattern;

                left = getParticle(crunchy.colorLeft, crunchy.distanceLeft);
                right = getParticle(crunchy.colorRight, crunchy.distanceRight);

                //            This should read like a CSS document, with the pattern being modified if the situation calls for it
                //            Essentially, the least important stuff is at the l=top and the most important stuff is at the bottom.

                boolean redAlliance = true;

                //stuff for the Red alliance which changes as time decreases
                if (redAlliance)
                {
                    pattern = COLOR_WAVES_LAVA_PALETTE;
                    if (time() >= 90) pattern = HEARTBEAT_RED;
                    if (time() >= 110) pattern = STROBE_RED;
                    if (time() >= 120) pattern = RAINBOW_WITH_GLITTER;
                }

                //stuff for the blue alliance which changes as time decreases
                if (!redAlliance)
                {
                    pattern = RAINBOW_OCEAN_PALETTE;
                    if (time() >= 90) pattern = HEARTBEAT_BLUE;
                    if (time() >= 110) pattern = STROBE_RED;
                    if (time() >= 120) pattern = RAINBOW_WITH_GLITTER;
                }

                //code for the particles that should only take affect when the intake is down
                if (!intakeTargetIsUp)
                {
                    //if one of the particles is yellow then strobe yellow.
                    if ((left == Particle.YELLOW && right == Particle.NONE) || (left == Particle.NONE && right == Particle.YELLOW))
                    {
                        pattern = STROBE_GOLD;
                    }
                    //if one of the particles is white then strobe white
                    else if ((left == Particle.WHITE && right == Particle.NONE) || (left == Particle.NONE && right == Particle.WHITE))
                    {
                        pattern = STROBE_WHITE;
                    }
                    //if particles are different then flash a gradient of the two colors
                    else if ((left == Particle.YELLOW && right == Particle.WHITE) || (left == Particle.WHITE && right == Particle.YELLOW))
                    {
                        pattern = CP1_2_COLOR_GRADIENT;
                    }
                    //if the are both yellow, then remain yellow
                    else if (left == Particle.YELLOW && right == Particle.YELLOW)
                    {
                        pattern = GOLD;
                    }
                    //if they are both white then remain white
                    else if (left == Particle.WHITE && right == Particle.WHITE)
                    {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                    }
                }

                if(pattern != oldPattern)
                {
                    displayPattern();
                }
            }

            telemetry.update();
        }
    }

    //also probably needs to go into crunchy
    private void displayPattern() {
        crunchy.blinkinLedDriver.setPattern(pattern);
    }

    private PropertiesLoader colorSensorCalibration = new PropertiesLoader("ColorSensorCalibration");
    private double DISTANCE_THRESHOLD_LEFT = colorSensorCalibration.getDoubleProperty("distanceThresholdLeft");
    private double DISTANCE_THRESHOLD_RIGHT = colorSensorCalibration.getDoubleProperty("distanceThresholdRight");
    private double DISTANCE_THRESHOLD = (DISTANCE_THRESHOLD_LEFT + DISTANCE_THRESHOLD_RIGHT) / 2;
    private double COLOR_THRESHOLD_LEFT = colorSensorCalibration.getDoubleProperty("colorThresholdLeft");
    private double COLOR_THRESHOLD_RIGHT = colorSensorCalibration.getDoubleProperty("colorThresholdRight");
    private double COLOR_THRESHOLD = (COLOR_THRESHOLD_LEFT + COLOR_THRESHOLD_RIGHT) / 2;

    private Particle getParticle(ColorSensor cs, DistanceSensor ds)
    {
        double distance = ds.getDistance(DistanceUnit.MM);
        double blue = cs.blue();

        if(Double.isNaN(distance) || distance > DISTANCE_THRESHOLD)
        {
            return Particle.NONE;
        }
        else
        {
            if(blue > COLOR_THRESHOLD)
            {
                return Particle.WHITE;
            }
            else
            {
                return Particle.YELLOW;
            }
        }
    }
    //three little methods for processing the imput of the color sensor
    //they also can go into crunchy, along w the enum to make things easier
    @Deprecated
    private boolean isYellow(int r, int g, int b) {
        //rgb(252, 200, 30) is a similar color to the yellow cube
        //given that its on a black background it doesn't seem like its too big of a deal

        boolean isYellow = true;

        if(!colorCheck(r,252,50))isYellow = false;
        else if(!colorCheck(g,200,50))isYellow = false;
        else if(!colorCheck(b,30,50))isYellow = false;

        return isYellow;
    }

    @Deprecated
    private boolean isWhite(int r, int g, int b) {
        //rgb(249, 242, 217) is a similar color to the white ball
        //given that its on a black background it doesn't seem like its too big of a deal

        boolean isWhite = true;

        if(!colorCheck(r,249,50))isWhite = false;
        else if(!colorCheck(g,242,50))isWhite = false;
        else if(!colorCheck(b,217,50))isWhite = false;

        return isWhite;
    }
    private boolean colorCheck(int maybe, int color, int range) {
        int upper = 0;
        int lower = 0;

        //puts a ceiling on the highest possible value that the upper limit acn be
        if (color + range > 255) upper = 255;
        else upper = color + range;

        //puts a floor on the lowest possible value that the lower limit can be
        if (color - range > 0) lower = 0;
        else lower = color - range;

        //checks to see if its in between the 2, returns true if it is.
        if (maybe >= lower && maybe <= upper) return true;
        else return false;
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

    private void togglePositionSlowly(Servo servo, double disengagedPosition, double engagedPosition, boolean button){
        //default state, when the button isn't pressed it remains its engaged.
        double targetPosition = button ? engagedPosition : disengagedPosition;
        double currentPosition = servo.getPosition();

        if (currentPosition < targetPosition)
        {
            if (currentPosition + SLOW_SERVO_SPEED >= targetPosition)
            {
                servo.setPosition(targetPosition);
            }
            else
            {
                servo.setPosition(currentPosition + SLOW_SERVO_SPEED);
            }
        }
        else
        {
            if (currentPosition - SLOW_SERVO_SPEED <= targetPosition)
            {
                servo.setPosition(targetPosition);
            }
            else
            {
                servo.setPosition(currentPosition - SLOW_SERVO_SPEED);
            }
        }
    }

    private boolean anyDPad()
    {
        return gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right;
    }

    private void RunIntakeLift () {
        double intakeliftUp = 1;//power level for going up
        double intakeliftDown = -0.4;//for going down

        int intakeUpperLimit = 0;//for the robot intake all the way in this is how the game starts
        int intakeLowerLimit = -1800;       //if the lift is the above where is should be
//for the robot intake all the way extended, when picking up particles.


        //intake toggle controls to change where we want to go
        intakeCurrState = gamepad2.x;

        if ((gamepad2.x) && (intakeCurrState != intakePrevState))
        {
            intakeTargetIsUp = !intakeTargetIsUp;

//            if(intakeTargetIsUp && crunchy.liftLeft.getCurrentPosition() > crunchy.LIFT_LOWER_LIMIT && hookIsOpen)
//            {
//                intakeTargetIsUp = false;
//            }
        }
        intakePrevState = intakeCurrState;

        if ((intakeTargetIsUp) && (crunchy.intakeLift.getCurrentPosition() < intakeUpperLimit))
        {
            if(crunchy.liftLeft.getCurrentPosition() > crunchy.LIFT_LOWER_LIMIT && hookIsOpen)
            {
                crunchy.liftLeft.setPower(-0.5);
                crunchy.liftRight.setPower(-0.5);
            }
            else
            {
                crunchy.intakeLift.setPower(intakeliftUp);
                crunchy.intake.setPower(-1);
            }
        }
        else if ((!intakeTargetIsUp) && (crunchy.intakeLift.getCurrentPosition() > intakeLowerLimit))
        {
            crunchy.intakeLift.setPower(intakeliftDown);
        }
        else crunchy.intakeLift.setPower(0);
    }

    private void RunLift () {
        int liftUpperLimit = crunchy.LIFT_UPPER_LIMIT;//I just did the math for the values because android studio got mad
        int liftLowerLimit = crunchy.LIFT_LOWER_LIMIT;//the lift is all the way down, the plus is to compensate for lag.

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
        else if (crunchy.liftLeft.getCurrentPosition() < liftLowerLimit + 300 && gamepad2.left_stick_y < 0)
        {
            crunchy.liftLeft.setPower(gamepad2.left_stick_y / 2);
            crunchy.liftRight.setPower(gamepad2.left_stick_y / 2);
        }
        else if (crunchy.liftLeft.getCurrentPosition() > liftUpperLimit - 300  && gamepad2.left_stick_y > 0)
        {
            crunchy.liftLeft.setPower(gamepad2.left_stick_y / 2);
            crunchy.liftRight.setPower(gamepad2.left_stick_y / 2);
        }
        else{
            crunchy.liftLeft.setPower(gamepad2.left_stick_y);
            crunchy.liftRight.setPower(gamepad2.left_stick_y);
        }
    }

    private void RunDrive () {
        double y = (((-gamepad1.left_stick_y)*(abs(-gamepad1.left_stick_y))+((-gamepad1.right_stick_y)*(abs(-gamepad1.right_stick_y))))/2);
        double x = (((-gamepad1.left_stick_x)*(abs(-gamepad1.left_stick_x))+((-gamepad1.right_stick_x)*(abs(-gamepad1.right_stick_x))))/2);
        double turnVal = (((-gamepad1.left_stick_y)-(-gamepad1.right_stick_y))/2);

        if(gamepad1.right_bumper) {
            y *= SLOW_MULTIPLIER;
            x *= SLOW_MULTIPLIER;
            turnVal *= abs(SLOW_MULTIPLIER);
        }

        crunchy.frontLeft.setPower(y-x+turnVal);
        crunchy.backLeft.setPower(y+x+turnVal);
        crunchy.frontRight.setPower(y+x-turnVal);
        crunchy.backRight.setPower(y-x-turnVal);

//        telemetry.addData("y", y);
//        telemetry.addData("x", x);
//        telemetry.addData("turnval", turnVal);
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
        double loDown = crunchy.LEFT_OUTPUT_DOWN;
        double loUp = crunchy.LEFT_OUTPUT_UP;
        double roDown = 1- loDown;
        double roUp = 1-loUp;

        double liftHalfway = 3100;

        boolean bucketControl = gamepad2.a;
        //controls of the bucket with gamapad1.a and makes sure that the user can't break the bot. It holds at a diagonal position when it can so that it keeps the particles
        if ((bucketControl == false) && crunchy.liftLeft.getCurrentPosition() > liftHalfway){
            crunchy.leftOutput.setPosition(0.35);
            crunchy.rightOutput.setPosition(0.65);
        }
        else if (crunchy.liftLeft.getCurrentPosition() < liftHalfway){
            togglePosition(crunchy.leftOutput, loDown, (loDown+0.1), bucketControl);//lets the user move the bucket around but only a little bit
            togglePosition(crunchy.rightOutput, roDown, (roDown-0.1), bucketControl);
        }
        else{

            togglePositionSlowly(crunchy.leftOutput, loDown, loUp, bucketControl);//full range of motion for scoring when the bucket is in the air
            togglePositionSlowly(crunchy.rightOutput, roDown, roUp, bucketControl);
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

        //makes it stop when the motor is at rest
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
