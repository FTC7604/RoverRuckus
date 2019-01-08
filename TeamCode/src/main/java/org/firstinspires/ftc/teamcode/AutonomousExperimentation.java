package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.CrunchyAutonomous;
import org.firstinspires.ftc.teamcode.util.DWAILinearOpMode;

@TeleOp(name = "random autonomous experiments")
public class AutonomousExperimentation extends DWAILinearOpMode
{
    private CrunchyAutonomous crunchy;

    @Override
    public void initOpMode() throws InterruptedException
    {
        crunchy = new CrunchyAutonomous(this);
    }

    @Override
    public void mainOpMode() throws InterruptedException
    {
        crunchy.sampleArm.setPosition(crunchy.SAMPLE_ARM_UP);
        sleep(2000);
        crunchy.sampleArm.setPosition(crunchy.SAMPLE_ARM_DOWN);
        sleep(2000);
    }

    @Override
    public void stopOpMode() throws InterruptedException
    {

    }
}
