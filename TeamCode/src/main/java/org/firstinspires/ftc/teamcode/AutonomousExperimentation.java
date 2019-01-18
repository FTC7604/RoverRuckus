package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.CrunchyAutonomous;
import org.firstinspires.ftc.teamcode.util.DWAILinearOpMode;
import org.firstinspires.ftc.teamcode.util.PropertiesLoader;

@TeleOp(name = "random autonomous experiment")
public class AutonomousExperimentation extends DWAILinearOpMode
{
    private CrunchyAutonomous crunchy;

    @Override
    public void initOpMode() throws InterruptedException
    {
        crunchy = new CrunchyAutonomous(this);

        crunchy.intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        crunchy.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        crunchy.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        crunchy.intakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        crunchy.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        crunchy.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void mainOpMode() throws InterruptedException
    {
        setIntakeLiftPosition(false);
        sleep(2000);
        setIntakeLiftPosition(true);
        sleep(2000);

    }

    private void setIntakeLiftPosition(boolean intakeTargetIsUp)
    {
        double intakeLiftUp = 1;//power level for going up
        double intakeLiftDown = -0.4;//for going down

        int intakeUpperLimit = 0;//for the robot intake all the way in this is how the game starts
        int intakeLowerLimit = -1800;

        while (ensureOpModeIsActive())
        {
            if ((intakeTargetIsUp) && (crunchy.intakeLift.getCurrentPosition() < intakeUpperLimit))
            {
                crunchy.intakeLift.setPower(intakeLiftUp);
            } else if ((!intakeTargetIsUp) && (crunchy.intakeLift.getCurrentPosition() > intakeLowerLimit))
            {
                crunchy.intakeLift.setPower(intakeLiftDown);
            } else
            {
                crunchy.intakeLift.setPower(0);
                break;
            }
        }
    }

    @Override
    public void stopOpMode() throws InterruptedException
    {

    }
}
