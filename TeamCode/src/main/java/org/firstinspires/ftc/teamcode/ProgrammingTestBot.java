package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Programming Test")
public class ProgrammingTestBot extends OpMode
{
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor upgoer;
    Servo top, bottom;

    @Override
    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        upgoer = hardwareMap.dcMotor.get("up");
        top = hardwareMap.servo.get("top");
        bottom = hardwareMap.servo.get("bottom");
    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {
        top.setPosition(0);
        bottom.setPosition(1);
    }

    @Override
    public void loop()
    {
        double leftDrive = gamepad1.left_stick_y;
        double rightDrive = gamepad1.right_stick_y;

        if (Math.abs(leftDrive) < 0.05)
        {
            leftDrive = 0;
        }

        if (Math.abs(rightDrive) < 0.05)
        {
            rightDrive = 0;
        }

        frontLeft.setPower(-leftDrive);
        backLeft.setPower(-leftDrive);

        frontRight.setPower(rightDrive);
        backRight.setPower(rightDrive);

        if (gamepad1.a) {
            top.setPosition(1);
            bottom.setPosition(0);
        }
        if (gamepad1.b) {
            top.setPosition(0);
            bottom.setPosition(1);
        }

        if(gamepad1.right_trigger > 0.4) {
            upgoer.setPower(-1);
        }
        else if(gamepad1.left_trigger > 0.4) {
            upgoer.setPower(1);
        }
        else {
            upgoer.setPower(0);
        }
    }

    @Override
    public void stop()
    {
    }
}