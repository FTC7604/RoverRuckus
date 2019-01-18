package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.CrunchyAutonomous;
import org.firstinspires.ftc.teamcode.util.DWAILinearOpMode;
import org.firstinspires.ftc.teamcode.util.PropertiesLoader;

@TeleOp(name = "turn test")
public class TurnTest extends DWAILinearOpMode
{
    private CrunchyAutonomous crunchy;

    private PropertiesLoader loader = new PropertiesLoader("TurnTest");
    private final double TURN_PRECISION_DEGREES = loader.getDoubleProperty("turnPrecisionDegrees");
    private final double ANGLE = loader.getDoubleProperty("angle");

    @Override
    public void initOpMode() throws InterruptedException
    {
        crunchy = new CrunchyAutonomous(this);
    }

    @Override
    public void mainOpMode() throws InterruptedException
    {
        crunchy.turnDegrees(ANGLE, TURN_PRECISION_DEGREES);
    }

    @Override
    public void stopOpMode() throws InterruptedException
    {

    }
}
