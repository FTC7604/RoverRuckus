package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    public BNO055IMU imu1, imu2;

    public DWAILinearOpMode opMode;

    private PropertiesLoader loader = new PropertiesLoader("Crunchy");
    public final double HOOK_OPEN = loader.getDoubleProperty("hookOpen");
    public final double HOOK_ENGAGED = loader.getDoubleProperty("hookEngaged");
    public final double LEFT_OUTPUT_DOWN = loader.getDoubleProperty("loDown");
    public final double LEFT_OUTPUT_UP = loader.getDoubleProperty("loUp");
    public final double RIGHT_OUTPUT_DOWN = 1 - LEFT_OUTPUT_DOWN;
    public final double RIGHT_OUTPUT_UP = 1 - LEFT_OUTPUT_UP;
    public final double OPEN_PHONE = loader.getDoubleProperty("openPhone");
    public final double CLOSED_PHONE = loader.getDoubleProperty("closedPhone");

    private final double kP = loader.getDoubleProperty("pidProportional");
    private final double kI = loader.getDoubleProperty("pidIntegral");
    private final double kD = loader.getDoubleProperty("pidDifferential");
    private final double pidMult = loader.getDoubleProperty("pidMultiplier");

    public Crunchy(DWAILinearOpMode opMode)
    {
        mapHardware(opMode.hardwareMap);
        initIMU();
        this.opMode = opMode;
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

        //makes it stopAndResetEncoders when the motor is at rest
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

        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
    }

    private void initIMU()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu1.initialize(parameters);
        imu2.initialize(parameters);
        BNO055IMU.CalibrationData calibrationData1 = imu1.readCalibrationData();
        File file1 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration1.json");
        ReadWriteFile.writeFile(file1, calibrationData1.serialize());
        BNO055IMU.CalibrationData calibrationData2 = imu2.readCalibrationData();
        File file2 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration2.json");
        ReadWriteFile.writeFile(file2, calibrationData2.serialize());
    }

    private double getDriveEncoderValue()
    {
        int fl = abs(frontLeft.getCurrentPosition());
        int fr = abs(frontRight.getCurrentPosition());
        int bl = abs(backLeft.getCurrentPosition());
        int br = abs(backRight.getCurrentPosition());

        return (fl + fr + bl + br) / 4.0;
    }

    public void driveForwardForDistance(double power, int distance)
    {
        stopAndResetEncoders();
        power *= signum(distance);
        distance = abs(distance);

        double currentPosition;
        while (opMode.ensureOpModeIsActive() && (currentPosition = getDriveEncoderValue()) < distance)
        {
            double remaining = distance - currentPosition;
            double ratio = (remaining + distance) / (2.0 * distance);
            drive(power * ratio);
        }
        stopAndResetEncoders();
    }


    public void strafeRightForDistance(double power, int distance)
    {
        stopAndResetEncoders();
        power *= signum(distance);
        distance = abs(distance);

        double x = -power * abs(power);

        double currentPosition;
        while (opMode.ensureOpModeIsActive() && (currentPosition = getDriveEncoderValue()) < distance)
        {
            double remaining = distance - currentPosition;
            double ratio = (remaining + distance) / (2.0 * distance);
            double val = x * ratio;
            drive(-val, val, val, -val);
        }
        stopAndResetEncoders();
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

    /* Positive turn direction is right (clockwise) */
    public void turnDegrees(double turnAngle, double precision)
    {
        turnRadians(toRadians(turnAngle), toRadians(precision));
    }

    public void turnRadians(double turnAngle, double precision)
    {
        turnAngle *= -1;

        double[] position = getIMUPosition();
        double desiredAngle = turnAngle + position[0];
        PIDAngleControl pidControl = new PIDAngleControl();
        pidControl.startPID(desiredAngle);

        while(opMode.ensureOpModeIsActive() && abs(desiredAngle - position[0]) >= precision)
        {
            position = getIMUPosition();
            pidControl.onSensorChanged(position[0]);
            double turnVal = pidControl.getValue(kP, kI, kD, pidMult);
            opMode.telemetry.addData("konstants", kP + " " + kI + " " + kD + " " + pidMult);
            opMode.telemetry.addData("error", pidControl.getError());
            opMode.telemetry.addData("integral", pidControl.getIntegral());
            opMode.telemetry.addData("derivative", pidControl.getDerivative());
            opMode.telemetry.addData("turn", turnVal);
            opMode.telemetry.update();
            drive(turnVal, -turnVal);
        }

        stopAndResetEncoders();
    }
}
