package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.teamcode.util.CrunchyAutonomous;
import org.firstinspires.ftc.teamcode.util.DWAILinearOpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name = "Color Sensor Calibration")
public class ColorSensorCalibration extends DWAILinearOpMode
{
    private CrunchyAutonomous crunchy;

    @Override
    public void initOpMode() throws InterruptedException
    {
        crunchy = new CrunchyAutonomous(this);
        ((OpModeManagerImpl) internalOpModeServices).startActiveOpMode();
    }

    private boolean a()
    {
        return gamepad1.a || gamepad2.a;
    }

    private void waitForAPressed()
    {
        //noinspection StatementWithEmptyBody
        while (ensureOpModeIsActive() && !a()) ;
        //noinspection StatementWithEmptyBody
        while (ensureOpModeIsActive() && a()) ;
    }

    @Override
    public void mainOpMode() throws InterruptedException
    {
        final String pressAText = "Then, press A to set calibration.";

        telemetry.addLine("Empty both slots.");
        telemetry.addLine("Then, press A to set calibration.");
        telemetry.update();
        waitForAPressed();
        double emptyDistanceLeft = crunchy.distanceLeft.getDistance(DistanceUnit.MM);
        double emptyDistanceRight = crunchy.distanceRight.getDistance(DistanceUnit.MM);

        telemetry.addLine("Place a gold mineral in both slots.");
        telemetry.addLine("Then, press A to set calibration.");
        telemetry.update();
        waitForAPressed();

        double goldColorLeft = crunchy.colorLeft.blue();
        double goldColorRight = crunchy.colorRight.blue();
        double goldDistanceLeft = crunchy.distanceLeft.getDistance(DistanceUnit.MM);
        double goldDistanceRight = crunchy.distanceRight.getDistance(DistanceUnit.MM);


        telemetry.clear();
        telemetry.addLine("Place a silver mineral in both slots.");
        telemetry.addLine("Then, press A to set calibration.");
        telemetry.update();

        waitForAPressed();

        double silverColorLeft = crunchy.colorLeft.blue();
        double silverColorRight = crunchy.colorRight.blue();
        double silverDistanceLeft = crunchy.distanceLeft.getDistance(DistanceUnit.MM);
        double silverDistanceRight = crunchy.distanceRight.getDistance(DistanceUnit.MM);

        double mineralDistanceLeft = Math.max(goldDistanceLeft, silverDistanceLeft);
        double mineralDistanceRight = Math.max(goldDistanceRight, silverDistanceRight);
        double distanceThresholdLeft = (mineralDistanceLeft + emptyDistanceLeft) / 2;
        double distanceThresholdRight = (mineralDistanceRight + emptyDistanceRight) / 2;

        double colorThresholdLeft = (goldColorLeft + silverColorLeft) / 2;
        double colorThresholdRight = (goldColorRight + silverColorRight) / 2;

        distanceThresholdLeft = distanceThresholdRight = 100;

        File sdcard = Environment.getExternalStorageDirectory();
        File config = new File(sdcard, "DWAIConfig/ColorSensorCalibration.properties");
        try
        {
            FileWriter out = new FileWriter(config, false);
            //noinspection SpellCheckingInspection
            out.append(String.format(Locale.US,
                    "distanceThresholdLeft=%f%n" +
                            "distanceThresholdRight=%f%n" +
                            "colorThresholdLeft=%f%n" +
                            "colorThresholdRight=%f",
                    distanceThresholdLeft,
                    distanceThresholdRight,
                    colorThresholdLeft,
                    colorThresholdRight));

            out.close();
        } catch (IOException e)
        {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void stopOpMode() throws InterruptedException
    {

    }
}
