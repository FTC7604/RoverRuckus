package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Crunchy;
import org.firstinspires.ftc.teamcode.util.MotorControl;
import org.firstinspires.ftc.teamcode.util.PropertiesLoader;


@Autonomous(name = "deploy Test", group = "Liner Op")
public class AutonomousPrototype extends LinearOpMode
{
    //creates the runtime
    private ElapsedTime runtime = new ElapsedTime();
    private Crunchy crunchy = new Crunchy();
    private MotorControl motorControl = new MotorControl();

    private PropertiesLoader loader = new PropertiesLoader("AutonomousPrototype");
    private double hookOpen = loader.getDoubleProperty("hookOpen");
    private double hookEngaged = loader.getDoubleProperty("hookEngaged");
    private double loDown = loader.getDoubleProperty("loDown");
    private double loUp = loader.getDoubleProperty("loUp");
    private double roDown = 1 - loDown;
    private double roUp = 1 - loUp;
    private double liftPower = loader.getDoubleProperty("liftPower");

    int detectSample()
    {
        int position = 0;
        return position;
    }

    //Movement methods
    void deploy()
    {
        int liftUpperLimit = (3889);//I just did the math for the values because android studio got mad
        int liftLowerLimit = (0 + 75);//the lift is all the way down, the plus is to compensate for lag.

        telemetry.addLine("Deploying");
        telemetry.update();

        crunchy.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        crunchy.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorControl.init(crunchy.liftLeft, liftUpperLimit);

        //lower down then stop
        while (!motorControl.isCompleted(crunchy.liftLeft))
        {
            crunchy.liftLeft.setPower(liftPower);
            crunchy.liftRight.setPower(liftPower);
        }

        telemetry.clear();
        telemetry.addLine("DEPLOYING HAS CEASED");
        telemetry.update();

        crunchy.liftLeft.setPower(0);
        crunchy.liftRight.setPower(0);
    }

    void mineralArm(boolean deployed)
    {
        if (deployed)
        {
            //sampleArm.setPosition();
        } else
        {
            //sampleArm.setPosition();
        }

        telemetry.clear();
        telemetry.addLine("SAMPLE HAS HAS MOVED!");
        telemetry.update();
    }

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        crunchy.mapHardware(this);

        waitForStart();

        runtime.reset();
        telemetry.clearAll();

        deploy();

        crunchy.hook.setPosition(hookOpen);

        crunchy.drive(0.5, 0.5, 0.5, 0.5);
        motorControl.waitForDistance(crunchy.frontLeft, 950);
        crunchy.stop();

        int mineralPosition = 0;
        mineralPosition = detectSample();

        switch (mineralPosition)
        {
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