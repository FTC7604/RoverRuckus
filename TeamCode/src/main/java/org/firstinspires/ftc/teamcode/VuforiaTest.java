package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Crunchy;
import org.firstinspires.ftc.teamcode.util.PropertiesLoader;
import org.firstinspires.ftc.teamcode.util.vision.VisionTarget;
import org.firstinspires.ftc.teamcode.util.vision.VisionTracking;

import java.util.Locale;

/*@TeleOp(name = "Voo4ia test")
public class VuforiaTest extends LinearOpMode
{
    private PropertiesLoader loader = new PropertiesLoader("AutonomousPrototype");
    private double hookOpen = loader.getDoubleProperty("hookOpen");
    private Crunchy crunchy = new Crunchy();

    @Override
    public void runOpMode()
    {
        VisionTracking tracking = new VisionTracking(this);
        tracking.init();
        crunchy.mapHardware(this);

        waitForStart();

        crunchy.hook.setPosition(hookOpen);

        while(opModeIsActive())
        {
            tracking.updateTracking();

            for (VisionTarget target : tracking.getTrackingInfo())
            {
                String targetString = String.format(Locale.US,
                        "translation: [%f, %f, %f], rotation: [%f, %f, %f], age: %d",
                        target.translationX, target.translationY, target.translationZ,
                        target.yaw, target.pitch, target.roll,
                        target.age);

                telemetry.addData(target.name, targetString);
            }

            telemetry.update();
        }
    }
}*/