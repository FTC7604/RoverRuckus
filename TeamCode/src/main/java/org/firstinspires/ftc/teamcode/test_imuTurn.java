package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static java.lang.Math.*;

@Autonomous(name = "Auto turn", group = "Liner Op")

public class test_imuTurn extends LinearOpMode{
    //creates the runtime
    private ElapsedTime runtime = new ElapsedTime();

    //adds the imu methods
    private IMUControl IMUControl = new IMUControl();
    private PIDAngleControl PIDControl = new PIDAngleControl();

    //drivetrain
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack    = null;
    private DcMotorEx rightBack   = null;

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

    private BNO055IMU imu1 = null;
    private BNO055IMU imu2 = null;

    //hook
    private static final double hookOpen = 0.2;
    private static final double hookEngaged = 0.6;

    //ScoringBucket
    private static final double loDown = 0;
    private static final double loUp = 0.75;
    private static final double roDown = 1 - loDown;
    private static final double roUp = 1 - loUp;

    //Hardware mapping
    void setUP(){
        createHardwareMap();
        setDirections();
        setEncoders();
        setBrakes();
    }

    void createHardwareMap(){
        //drivetrain
        leftFront  = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        rightFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        leftBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        rightBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");

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

        //imus
        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
    }
    void setDirections(){
        //sets the direction for the lift
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        //sets the direction for the intake
        intake.setDirection(DcMotor.Direction.FORWARD);
        intakeLift.setDirection(DcMotor.Direction.FORWARD);

        //sets the directioon for the drivetrain
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
    }
    void setEncoders(){
        //resets the aux encoders
        intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //resets the drive encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //starts the aux encoders
        intakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //resets the drive encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void setBrakes(){
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    double[] getDrivePosition(double[]imputs){
        imputs[0] = leftFront.getCurrentPosition();
        imputs[1] = leftBack.getCurrentPosition();
        imputs[2] = rightFront.getCurrentPosition();
        imputs[3] = rightBack.getCurrentPosition();
        return imputs;
    }

    //int detectSample(){}

    //Movement methods
    void Deploy() {
        int liftUpperLimit = (3530 - 100);//I just did the math for the values because android studio got mad
        int liftLowerLimit = (200);//the lift is all the way down, the plus is to compensate for lag.

        telemetry.addLine("Deploying");
        telemetry.update();

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setTargetPosition(liftUpperLimit);

        //lower down then stop
        while (leftLift.isBusy()){
            leftLift.setPower(1);
            rightLift.setPower(1);
        }

        telemetry.clear();
        telemetry.addLine("DEPLOYING HAS CEASED");
        telemetry.update();

        leftLift.setPower(0);
        rightLift.setPower(0);
    }

    void mineralArm(boolean deployed) {
        if (deployed) {
            //sampleArm.setPosition();
        } else{
            //sampleArm.setPosition();
        }

        telemetry.clear();
        telemetry.addLine("SAMPLE HAS HAS MOVED!");
        telemetry.update();
    }

    void getShitDone(double one, double two, double three, double four, int time){
        rightFront.setPower(one);
        leftFront.setPower(two);
        rightBack.setPower(three);
        leftBack.setPower(four);

        telemetry.addData("rightFront", one);
        telemetry.addData("leftFront", two);
        telemetry.addData("rightBack", three);
        telemetry.addData("leftBack", four);
        telemetry.update();

        sleep(time);

        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
    }

    void driveForward(double power, int time){
        getShitDone(power,power,power,power,time);
    }

    void driveStrafe(double power, int time){
        //getShitDone();
    }

    private void imputMecVelocity(double[]imputs){
        arrayScale(imputs,400);
        leftFront.setVelocity(imputs[0], AngleUnit.DEGREES);
        leftBack.setVelocity(imputs[1], AngleUnit.DEGREES);
        rightFront.setVelocity(imputs[2], AngleUnit.DEGREES);
        rightBack.setVelocity(imputs[3], AngleUnit.DEGREES);
    }
    private void imputMecPower(double[]imputs){
        leftFront.setPower(imputs[0]);
        leftBack.setPower(imputs[1]);
        rightFront.setPower(imputs[2]);
        rightBack.setPower(imputs[3]);
    }

    private void imputMecEncoders(int imput){
        leftFront.setTargetPosition(imput);
        leftBack.setTargetPosition(imput);
        rightFront.setTargetPosition(imput);
        rightBack.setTargetPosition(imput);
    }
    private boolean imputMecBusy(){
        boolean busy = true;

        if(!leftFront.isBusy() && !leftBack.isBusy() && !rightFront.isBusy() && !rightBack.isBusy()){
            busy = false;
        }

        return busy;
    }

    private double[] arrayScale(double[] imput, double scalar){
        for(int i = imput.length; i > 0; i--){
            imput[i-1] *= scalar;
        }
        return imput;
    }
    private double[] arrayAddDouble(double[]imput, double add){
        for(int i = imput.length - 1; i > -1; i--){
            imput[i] += add;
        }
        return imput;
    }
    private double arrayAverage(double[]imput){
        double arrayAverage = 0;

        for(int i = imput.length - 1; i > -1; i--){
            arrayAverage += imput[i];
        }

        arrayAverage /= imput.length;
        return arrayAverage;
    }
    private double[] arraySubtractArray(double[] imput1,double[] imput2){
        for(int i = imput1.length - 1; i > -1; i--){
            imput1[i] -= imput2[i];
        }

        return imput1;
    }
    private int[] doubleArrayToInt(double[] imput){
        int[] output = new int[4];
        for (int i = imput.length - 1; i > -1; i --){
            output[i] = (int) imput[i];
        }
        return output;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        setUP();
        waitForStart();

        runtime.reset();
        telemetry.clearAll();

        double startTime = time;

        IMUControl.createIMU(imu1, imu2);
        IMUControl.calibrateIMU(imu1, imu2);

        //imuTurn(PI/2,.0005);
        double[] motors = new double[4];
        double[] position = new double[3];

        double turn = PI/2;

        double beginTime = time;
        imuTurn(turn,0.02);
        double endTime = time;


        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        while(opModeIsActive() && time < 3000){
            telemetry.addLine("I'm done.");
            telemetry.addData("It took too long",endTime - beginTime);
            telemetry.update();
        }




        //sleep(1000);
    }

    void imuTurn(double turnAngle, double precision){
        double[] motors = new double[4];
        double[] imputs = new double[3];
        double[] position = new double[3];

        boolean angleCondition = false;
        boolean motorCondition = false;

        IMUControl.getPosition(position,imu1,imu2,true);
        double desiredTurnPosition = turnAngle + position[0];

        PIDControl.startPID(desiredTurnPosition);

        do{
            IMUControl.getPosition(position,imu1,imu2,false);
            PIDControl.onSensorChanged(position[0]);
            imputs[2] = PIDControl.getValue(2.9,1.5,.9,.6);

            IMUControl.imuDrive(motors,imputs,0,false,false);
            imputMecVelocity(motors);

            if(IMUControl.remainTurn(desiredTurnPosition,position[0]) < precision)angleCondition = true;
            else angleCondition = false;
            if(abs(imputs[2]) < 10 * precision) motorCondition = true;
            else motorCondition = false;

        }while(opModeIsActive() && !(angleCondition && motorCondition));
    }
}