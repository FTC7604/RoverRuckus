package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Crunchy;
import org.firstinspires.ftc.teamcode.util.MotorControl;
import org.firstinspires.ftc.teamcode.util.PropertiesLoader;
import org.firstinspires.ftc.teamcode.util.vision.VisionTarget;
import org.firstinspires.ftc.teamcode.util.vision.VisionTracking;

import java.lang.annotation.Target;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;


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
    private double openPhone = loader.getDoubleProperty("openPhone");
    private double closedPhone = loader.getDoubleProperty("closedPhone");

    private enum SamplePosition
    {
        LEFT, CENTER, RIGHT
    }

    private SamplePosition detectSample(VisionTracking tracking) {
        runtime.reset();
        int mineralPosition = 0;

        while ((mineralPosition == 0 || runtime.seconds() < 2) && opModeIsActive()) {
            telemetry.clearAll();
            telemetry.addLine("Looping " + getRuntime());
            mineralPosition = tracking.detectMineral();
            telemetry.update();
        }

        telemetry.clearAll();

        switch(mineralPosition){
            case 1:
                return SamplePosition.LEFT;
            case 2:
                return SamplePosition.CENTER;
            case 3:
                return SamplePosition.RIGHT;
            default:
                telemetry.addLine("Error while detecting");
                return SamplePosition.CENTER;
        }
    }

    //Movement methods
    private void deploy()
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

    private void mineralArm(boolean deployed)
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
        //Setting up all processes
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        crunchy.mapHardware(this);

        final VisionTracking tracking = new VisionTracking(this);
        tracking.init();

        crunchy.phoneMount.setPosition(closedPhone);
        tracking.initTfod();

        waitForStart();

        //Autonomous code
        runtime.reset();
        telemetry.clearAll();

        crunchy.phoneMount.setPosition(openPhone);

        SamplePosition mineralPosition = detectSample(tracking);
        tracking.shutdownTfod();

        crunchy.phoneMount.setPosition(closedPhone);

        telemetry.addData("Sample position", mineralPosition);
        telemetry.update();

        tracking.initVision();

        /*deploy();

        crunchy.hook.setPosition(hookOpen);

        crunchy.drive(0.5, 0.5, 0.5, 0.5);
        motorControl.waitForDistance(crunchy.frontLeft, 950);
        crunchy.stop();

        while (opModeIsActive())
        {
            tracking.updateTracking();
            Collection<VisionTarget> targets = tracking.getTrackingInfo();

            
        }


        telemetry.clearAll();
        telemetry.addLine("This thing is done");
        telemetry.update();*/

        sleep(1000);
    }
}