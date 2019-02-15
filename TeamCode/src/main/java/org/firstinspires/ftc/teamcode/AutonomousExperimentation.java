package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.CrunchyAutonomous;
import org.firstinspires.ftc.teamcode.util.DWAILinearOpMode;

@TeleOp(name = "servo test")
public class AutonomousExperimentation extends DWAILinearOpMode
{
    private CrunchyAutonomous crunchy;

    private boolean previous = false;
    private boolean open = false;

    @Override
    public void initOpMode() throws InterruptedException
    {
        crunchy = new CrunchyAutonomous(this);
        crunchy.marker.setPosition(crunchy.MARKER_CLOSED);
    }

    @Override
    public void mainOpMode() throws InterruptedException
    {
        while (ensureOpModeIsActive())
        {
            //this block of code controls the position of the hook with a toggle switch.
            boolean current = gamepad2.y;
            //toggle the hook position with gamepad2.y
            if ((gamepad2.y) && current != previous)
            {
                open = !open;//change the hook target
            }
            previous = current;

            if (open) crunchy.marker.setPosition(crunchy.MARKER_OPEN);
            else crunchy.marker.setPosition(crunchy.MARKER_CLOSED);
        }
    }

    @Override
    public void stopOpMode() throws InterruptedException
    {

    }
}
