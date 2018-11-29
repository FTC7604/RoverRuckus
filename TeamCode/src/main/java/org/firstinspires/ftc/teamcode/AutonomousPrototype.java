package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Deploy Test", group = "Liner Op")
public class AutonomousPrototype extends LinearOpMode{
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
    private static final double roDown = 1 - loDown;
    private static final double roUp = 1 - loUp;

    //mineral arm servos
    private static final double downPos = 0;
    private static final double upPos = 0;

    //Hardware mapping
    private void mapHardware(){
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
    }

    private int detectSample(){
        int position = 0;

        /*if(){
           return 1;
        } else if(){
            return 3;
        } else{
            return 2;
        }*/

        return position;
    }

    //Movement methods
    private void Deploy() {
        int liftUpperLimit = 3889;//I just did the math for the values because android studio got mad
        int liftLowerLimit = 75;//the lift is all the way down, the plus is to compensate for lag.

        telemetry.addLine("Deploying");
        telemetry.update();

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setTargetPosition(liftUpperLimit);
        rightLift.setTargetPosition(liftUpperLimit);

        //lower down then stop
        while (leftLift.isBusy() && rightLift.isBusy()){
            leftLift.setPower(1);
            rightLift.setPower(1);
        }

        telemetry.clear();
        telemetry.addLine("DEPLOYING HAS CEASED");
        telemetry.update();

        leftLift.setPower(0);
        rightLift.setPower(0);
    }

    private void mineralArm(boolean deployed) {
        if (deployed) {
            //sampleArm.setPosition();
        } else{
            //sampleArm.setPosition();
        }

        telemetry.clear();
        telemetry.addLine("SAMPLE HAS HAS MOVED!");
        telemetry.update();
    }

    private void getShitDone(double one, double two, double three, double four, int time){
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

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        mapHardware();
        waitForStart();
        runtime.reset();
        telemetry.clearAll();

        Deploy();

        int mineralPosition = 0;
        mineralPosition = detectSample();

        switch(mineralPosition){
            case 1:
                //Rotate 90 degrees left
                //Strafe right
                //Move servo arm down
                //Move forwards
                break;
            case 2:
                //Drive straight forwards
                break;
            case 3:
                //Rotate 90 degrees left
                //Move backwards
                //Move servo arm down
                //Strafe right
                //Move forwards
                break;
        }

        telemetry.clearAll();
        telemetry.addLine("This thing is done");
        telemetry.update();

        sleep(1000);
    }
}