package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.CancellationException;

public abstract class DWAILinearOpMode extends LinearOpMode
{
    private String lastStep = null;
    private boolean stepMode;

    protected DWAILinearOpMode()
    {
        this(false);
    }

    protected DWAILinearOpMode(boolean stepMode)
    {
        this.stepMode = stepMode;
    }

    protected void enableSteps()
    {
        stepMode = true;
    }

    protected void step(String name)
    {
        if (!stepMode)
        {
            return;
        }

        if (lastStep != null)
        {
            telemetry.addLine("Completed step: " + lastStep);
            telemetry.addLine("Next step: " + name);
            telemetry.addLine("Press A to continue");
            telemetry.update();

            //noinspection StatementWithEmptyBody
            while (!gamepad1.a && !gamepad2.a && ensureOpModeIsActive());

            telemetry.clear();
        }

        lastStep = name;
    }

    public boolean ensureOpModeIsActive()
    {
        if (opModeIsActive())
        {
            return true;
        }

        throw new CancellationException();
    }

    @Override
    public final void runOpMode() throws InterruptedException
    {
        try
        {
            initOpMode();
            waitForStart();
            mainOpMode();
        } catch (Exception e)
        {
            if(isStopRequested())
            {
                stopOpMode();
            }
            throw e;
        }
    }

    public abstract void initOpMode() throws InterruptedException;

    public abstract void mainOpMode() throws InterruptedException;

    public abstract void stopOpMode() throws InterruptedException;
}
