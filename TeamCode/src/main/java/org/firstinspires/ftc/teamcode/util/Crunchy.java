package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Crunchy
{
    //drivetrain
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    //lift motors
    public DcMotor liftLeft = null;//looking dead on at the front of the bot
    public DcMotor liftRight = null;

    //intake motors
    public DcMotor intake = null;
    public DcMotor intakeLift = null;

    //all of the servos
    public Servo hook = null;
    public Servo phoneMount = null;
    public Servo sampleArm = null;
    public Servo leftOutput = null;
    public Servo rightOutput = null;


    public void mapHardware(OpMode opmode) {
        mapHardware(opmode.hardwareMap);
    }

    //Hardware mapping
    public void mapHardware(HardwareMap hardwareMap){
        //drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backLeft = hardwareMap.get(DcMotor.class, "lb");
        backRight = hardwareMap.get(DcMotor.class, "rb");

        //output and intake
        liftLeft = hardwareMap.get(DcMotor.class, "ll");
        liftRight = hardwareMap.get(DcMotor.class, "rl");
        intake = hardwareMap.get(DcMotor.class, "in");
        intakeLift = hardwareMap.get(DcMotor.class, "il");

        //servos
        hook = hardwareMap.get(Servo.class, "hk");
        phoneMount = hardwareMap.get(Servo.class, "ph");
        sampleArm = hardwareMap.get(Servo.class, "sa");
        leftOutput = hardwareMap.get(Servo.class, "lo");
        rightOutput = hardwareMap.get(Servo.class, "ro");

        //sets the direction for the lift
        liftLeft.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setDirection(DcMotor.Direction.REVERSE);

        //sets the direction for the intake
        intake.setDirection(DcMotor.Direction.FORWARD);
        intakeLift.setDirection(DcMotor.Direction.FORWARD);

        //resets all the encoders
        intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //starts all the encoders
        intakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //makes it stop when the motor is at rest
        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //sets the directioon for the drivetrain
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        //when its 0 brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double fl, double fr, double bl, double br)
    {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void drive(double left, double right)
    {
        drive(left, right, left, right);
    }

    public void drive(double power)
    {
        drive(power, power);
    }

    public void stop()
    {
        drive(0);
    }
}
