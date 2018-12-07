package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mec turn test", group = "Liner Op")
public class test_imuTurn extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //creates the hardwaremap
        leftFront  = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        //the imu
        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        //Other imu
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

        double[]position = new double[3];

        IMUControl.getPosition(position,imu1,imu2);

        double startPosition = position[0];
        double desiredPosition = startPosition + Math.PI/2;

        double currentPosition = 0;

        double[]imput = new double[3];
        double[]motors = new double[3];

        imput[0] = 0;
        imput[1] = 0;

        while(desiredPosition > currentPosition - .1 && desiredPosition < currentPosition + .1){
            IMUControl.getPosition(position,imu1,imu2);
            currentPosition = position[0];

            imput[2] = desiredPosition - currentPosition;
            IMUControl.imuDrive(motors,imput,0,false);

            leftFront.setPower(motors[0]);
            leftBack.setPower(motors[1]);
            rightFront.setPower(motors[2]);
            rightBack.setPower(motors[3]);
        }
    }


}
