package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PIDAngleControl;

import java.io.File;

import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static java.lang.Math.toRadians;

public class Crunchy
{
    //drivetrain
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;

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
    public Servo marker = null;

    public BNO055IMU imu1, imu2;

    public ColorSensor colorLeft, colorRight;
    public DistanceSensor distanceLeft, distanceRight;

    public RevBlinkinLedDriver blinkinLedDriver;

    private PropertiesLoader loader = new PropertiesLoader("Crunchy");
    protected final double VELOCITY_MODE = loader.getDoubleProperty("velocityMode");

    public final double HOOK_OPEN = loader.getDoubleProperty("hookOpen");
    public final double HOOK_ENGAGED = loader.getDoubleProperty("hookEngaged");
    public final double LEFT_OUTPUT_DOWN = loader.getDoubleProperty("loDown");
    public final double LEFT_OUTPUT_UP = loader.getDoubleProperty("loUp");
    public final double RIGHT_OUTPUT_DOWN = 1 - LEFT_OUTPUT_DOWN;
    public final double RIGHT_OUTPUT_UP = 1 - LEFT_OUTPUT_UP;
    public final int LIFT_UPPER_LIMIT = loader.getIntegerProperty("liftUpperLimit");
    public final int LIFT_LOWER_LIMIT = loader.getIntegerProperty("liftLowerLimit");
    public final double OPEN_PHONE = loader.getDoubleProperty("openPhone");
    public final double CLOSED_PHONE = loader.getDoubleProperty("closedPhone");
    public final double SAMPLE_ARM_UP = loader.getDoubleProperty("sampleArmUp");
    public final double SAMPLE_ARM_DOWN = loader.getDoubleProperty("sampleArmDown");
    public final double MARKER_OPEN = loader.getDoubleProperty("markerOpen");
    public final double MARKER_CLOSED = loader.getDoubleProperty("markerClosed");


    protected final boolean PID_ENABLED = loader.getBooleanProperty("pidEnabled");
    protected final double PID_PRECISION_THRESHOLD_MULT = loader.getDoubleProperty("pidPrecisionThresholdMult");
    protected final double kP = loader.getDoubleProperty("pidProportional");
    protected final double kI = loader.getDoubleProperty("pidIntegral");
    protected final double kD = loader.getDoubleProperty("pidDifferential");
    protected final double PID_MULT = loader.getDoubleProperty("pidMultiplier");
    protected final double PID_MIN_POWER = loader.getDoubleProperty("pidMinPower");
    protected final double PID_MAX_DIFFERENTIAL = loader.getDoubleProperty("pidMaxDifferential");

    public Crunchy(OpMode opMode)
    {
        mapHardware(opMode.hardwareMap);
        initIMU();
    }

    //Hardware mapping
    private void mapHardware(HardwareMap hardwareMap)
    {
        //drivetrain
        frontLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        frontRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        backLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        backRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");

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
        marker = hardwareMap.get(Servo.class, "mk");

        //sets the direction for the lift
        liftLeft.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setDirection(DcMotor.Direction.REVERSE);

        //sets the direction for the intake
        intake.setDirection(DcMotor.Direction.FORWARD);
        intakeLift.setDirection(DcMotor.Direction.FORWARD);


        //makes it stopAndResetEncoders when the motor is at rest
        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //sets the direction for the drivetrain
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        //when its 0 brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");

        colorLeft = hardwareMap.get(ColorSensor.class, "cdl");
        colorRight = hardwareMap.get(ColorSensor.class, "cdr");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "cdl");
        distanceRight = hardwareMap.get(DistanceSensor.class, "cdr");

        //also need to be put in cruchy but the time in now
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "b");
    }

    private void initIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu1.initialize(parameters);
        imu2.initialize(parameters);

        File file1 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration1.json");
        ReadWriteFile.writeFile(file1, imu1.readCalibrationData().serialize());

        File file2 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration2.json");
        ReadWriteFile.writeFile(file2, imu2.readCalibrationData().serialize());
    }

    public double getDriveEncoderValue()
    {
        int fl = abs(frontLeft.getCurrentPosition());
        int fr = abs(frontRight.getCurrentPosition());
        int bl = abs(backLeft.getCurrentPosition());
        int br = abs(backRight.getCurrentPosition());

        return (fl + fr + bl + br) / 4.0;
    }

    public void drive(double fl, double fr, double bl, double br)
    {
        if(VELOCITY_MODE <= 0)
        {
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);
        }
        else
        {
            frontLeft.setVelocity(fl * VELOCITY_MODE);
            frontRight.setVelocity(fr * VELOCITY_MODE);
            backLeft.setVelocity(bl * VELOCITY_MODE);
            backRight.setVelocity(br * VELOCITY_MODE);
        }

    }

    public void drive(double left, double right)
    {
        drive(left, right, left, right);
    }

    public void drive(double power)
    {
        drive(power, power);
    }

    public void stopAndResetEncoders()
    {
        drive(0);
        setDriveMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDriveMotorRunMode(DcMotor.RunMode runMode)
    {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    public double[] getIMUPosition()
    {
        return new double[]
        {
            (imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle) / 2, //Yaw
            (imu1.getAngularOrientation().secondAngle + imu2.getAngularOrientation().secondAngle) / 2, //Roll
            (imu1.getAngularOrientation().thirdAngle + imu2.getAngularOrientation().thirdAngle) / 2 //Pitch
        };
    }

    public double getIMUYaw(){return (imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle)/2;}
    public double getIMURoll(){return (imu1.getAngularOrientation().secondAngle + imu2.getAngularOrientation().secondAngle)/2;}
    public double getIMUPitch(){return (imu1.getAngularOrientation().thirdAngle + imu2.getAngularOrientation().thirdAngle)/2;}
}
