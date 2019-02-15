package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.CancellationException;

public abstract class DWAILinearOpMode extends LinearOpMode
{
    private String lastStep = null;
    private long stepMode;

    protected DWAILinearOpMode()
    {
        this(0);
    }

    protected DWAILinearOpMode(long stepMode)
    {
        this.stepMode = stepMode;
    }

    protected void setStepMode(long stepMode)
    {
        this.stepMode = stepMode;
    }

    protected void step(String name)
    {
        if (stepMode == 0)
        {
            return;
        }

        if (stepMode > 0)
        {
            sleep(stepMode);
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

    public final void launchThread(final Runnable r)
    {
        new Thread()
        {
            @Override
            public void run()
            {
                try
                {
                    r.run();
                }
                catch (Exception e)
                {
                    // TODO: handle exception
                }
            }
        }.start();
    }

    public abstract void initOpMode() throws InterruptedException;

    public abstract void mainOpMode() throws InterruptedException;

    public abstract void stopOpMode() throws InterruptedException;
}
