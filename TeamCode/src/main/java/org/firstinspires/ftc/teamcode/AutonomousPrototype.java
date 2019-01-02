package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Crunchy;
import org.firstinspires.ftc.teamcode.util.DWAILinearOpMode;
import org.firstinspires.ftc.teamcode.util.PropertiesLoader;
import org.firstinspires.ftc.teamcode.util.vision.VisionTracking;

@TeleOp(name = "Autonomous")
public class AutonomousPrototype extends DWAILinearOpMode
{
    //creates the runtime
    private ElapsedTime runtime = new ElapsedTime();
    private Crunchy crunchy;

    private PropertiesLoader loader = new PropertiesLoader("AutonomousPrototype");
    private final boolean STEP_MODE = loader.getBooleanProperty("stepMode");
    private final double LIFT_POWER = loader.getDoubleProperty("liftPower");
    private final double TURN_PRECISION_DEGREES = loader.getDoubleProperty("turnPrecisionDegrees");
    private final int GOLD_POSITION = loader.getIntegerProperty("goldPosition");

    private final double RELEASE_AND_DRIVE_POWER = loader.getDoubleProperty("releaseAndDrivePower");
    private final int RELEASE_AND_DRIVE_DISTANCE = loader.getIntegerProperty("releaseAndDriveDistance");
    private final long DISPENSE_WAIT_TIME = loader.getLongProperty("dispenseWaitTime");

    private final double LEFT_BRANCH_STRAFE_TO_BALL_POWER = loader.getDoubleProperty("leftBranchStrafeToBallPower");
    private final int LEFT_BRANCH_STRAFE_TO_BALL_DISTANCE = loader.getIntegerProperty("leftBranchStrafeToBallDistance");
    private final double LEFT_BRANCH_HIT_BALL_POWER = loader.getDoubleProperty("leftBranchHitBallPower");
    private final int LEFT_BRANCH_HIT_BALL_DISTANCE = loader.getIntegerProperty("leftBranchHitBallDistance");
    private final double LEFT_BRANCH_TURN_TOWARDS_DEPOT_ANGLE = loader.getDoubleProperty("leftBranchTurnTowardsDepotAngle");
    private final double LEFT_BRANCH_DRIVE_TOWARDS_DEPOT_POWER = loader.getDoubleProperty("leftBranchDriveTowardsDepotPower");
    private final int LEFT_BRANCH_DRIVE_TOWARDS_DEPOT_DISTANCE = loader.getIntegerProperty("leftBranchDriveTowardsDepotDistance");

    private final double CENTER_BRANCH_STRAFE_TO_BALL_POWER = loader.getDoubleProperty("centerBranchStrafeToBallPower");
    private final int CENTER_BRANCH_STRAFE_TO_BALL_DISTANCE = loader.getIntegerProperty("centerBranchStrafeToBallDistance");
    private final double CENTER_BRANCH_HIT_BALL_POWER = loader.getDoubleProperty("centerBranchHitBallPower");
    private final int CENTER_BRANCH_HIT_BALL_DISTANCE = loader.getIntegerProperty("centerBranchHitBallDistance");
    private final double CENTER_BRANCH_TURN_TOWARDS_DEPOT_ANGLE = loader.getDoubleProperty("centerBranchTurnTowardsDepotAngle");

    private final double RIGHT_BRANCH_STRAFE_TO_BALL_POWER = loader.getDoubleProperty("rightBranchStrafeToBallPower");
    private final int RIGHT_BRANCH_STRAFE_TO_BALL_DISTANCE = loader.getIntegerProperty("rightBranchStrafeToBallDistance");
    private final double RIGHT_BRANCH_HIT_BALL_POWER = loader.getDoubleProperty("rightBranchHitBallPower");
    private final int RIGHT_BRANCH_HIT_BALL_DISTANCE = loader.getIntegerProperty("rightBranchHitBallDistance");
    private final double RIGHT_BRANCH_TURN_TOWARDS_WALL_ANGLE = loader.getDoubleProperty("rightBranchTurnTowardsWallAngle");
    private final double RIGHT_BRANCH_STRAFE_TOWARDS_DEPOT_POWER = loader.getDoubleProperty("rightBranchStrafeTowardsDepotPower");
    private final int RIGHT_BRANCH_STRAFE_TOWARDS_DEPOT_DISTANCE = loader.getIntegerProperty("rightBranchStrafeTowardsDepotDistance");

    private final double DRIVE_TOWARDS_CRATER_POWER = loader.getDoubleProperty("driveTowardsCraterPower");
    private final int DRIVE_TOWARDS_CRATER_DISTANCE = loader.getIntegerProperty("driveTowardsCraterDistance");

    private enum SamplePosition
    {
        LEFT, CENTER, RIGHT
    }

    private VisionTracking tracking;

    @Override
    public void initOpMode()
    {
        //Setting up all processes
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        crunchy = new Crunchy(this);

        tracking = new VisionTracking(this);
        tracking.init();

        crunchy.phoneMount.setPosition(crunchy.CLOSED_PHONE);
        tracking.initTfod();

        if (STEP_MODE)
        {
            enableSteps();
        }
    }

    @Override
    public void mainOpMode()
    {
        runtime.reset();
        telemetry.clearAll();

        step("Image detection");
        crunchy.phoneMount.setPosition(crunchy.OPEN_PHONE);

        SamplePosition mineralPosition;
        switch (GOLD_POSITION)
        {
            case 1:
                mineralPosition = SamplePosition.LEFT;
                break;
            case 2:
                mineralPosition = SamplePosition.CENTER;
                break;
            case 3:
                mineralPosition = SamplePosition.RIGHT;
                break;
            default:
                mineralPosition = detectSample(tracking);
                break;
        }
        tracking.shutdownTfod();

        crunchy.phoneMount.setPosition(crunchy.CLOSED_PHONE);

        telemetry.addData("Sample position", mineralPosition);
        telemetry.update();

        step(String.format("Detected %s - Deploy", mineralPosition.name()));
        deploy();

        step("Release and drive");
        crunchy.hook.setPosition(crunchy.HOOK_OPEN);
        crunchy.driveForwardForDistance(RELEASE_AND_DRIVE_POWER, RELEASE_AND_DRIVE_DISTANCE);

        switch (mineralPosition)
        {
            case LEFT:
                step("[Left Branch] Strafe to ball");
                crunchy.strafeRightForDistance(LEFT_BRANCH_STRAFE_TO_BALL_POWER, LEFT_BRANCH_STRAFE_TO_BALL_DISTANCE);
                step("[Left Branch] Hit ball");
                crunchy.driveForwardForDistance(LEFT_BRANCH_HIT_BALL_POWER, LEFT_BRANCH_HIT_BALL_DISTANCE);
                step("[Left Branch] Turn towards depot");
                crunchy.turnDegrees(LEFT_BRANCH_TURN_TOWARDS_DEPOT_ANGLE, TURN_PRECISION_DEGREES);
                step("[Left Branch] Drive towards depot");
                crunchy.driveForwardForDistance(LEFT_BRANCH_DRIVE_TOWARDS_DEPOT_POWER, LEFT_BRANCH_DRIVE_TOWARDS_DEPOT_DISTANCE);
                break;
            case CENTER:
                step("[Center Branch] Strafe to ball");
                crunchy.strafeRightForDistance(CENTER_BRANCH_STRAFE_TO_BALL_POWER, CENTER_BRANCH_STRAFE_TO_BALL_DISTANCE);
                step("[Center Branch] Hit ball");
                crunchy.driveForwardForDistance(CENTER_BRANCH_HIT_BALL_POWER, CENTER_BRANCH_HIT_BALL_DISTANCE);
                step("[Center Branch] Turn towards depot");
                crunchy.turnDegrees(CENTER_BRANCH_TURN_TOWARDS_DEPOT_ANGLE, TURN_PRECISION_DEGREES);
                break;
            case RIGHT:
                step("[Right Branch] Strafe to ball");
                crunchy.strafeRightForDistance(RIGHT_BRANCH_STRAFE_TO_BALL_POWER, RIGHT_BRANCH_STRAFE_TO_BALL_DISTANCE);
                step("[Right Branch] Hit ball");
                crunchy.driveForwardForDistance(RIGHT_BRANCH_HIT_BALL_POWER, RIGHT_BRANCH_HIT_BALL_DISTANCE);
                step("[Right Branch] Turn towards wall");
                crunchy.turnDegrees(RIGHT_BRANCH_TURN_TOWARDS_WALL_ANGLE, TURN_PRECISION_DEGREES);
                step("[Right Branch] Strafe towards depot");
                crunchy.strafeRightForDistance(RIGHT_BRANCH_STRAFE_TOWARDS_DEPOT_POWER, RIGHT_BRANCH_STRAFE_TOWARDS_DEPOT_DISTANCE);
                break;
        }

        step("Dispense");
        dispense();

        step("Drive towards crater");
        // TODO: lower intake lift while driving
        final int intakeLowerLimit = -1500;
        while(ensureOpModeIsActive() && (crunchy.intakeLift.getCurrentPosition() > intakeLowerLimit))
        {
            crunchy.intakeLift.setPower(1);
        }

        crunchy.intakeLift.setPower(0);
        crunchy.driveForwardForDistance(DRIVE_TOWARDS_CRATER_POWER, DRIVE_TOWARDS_CRATER_DISTANCE);

        telemetry.addLine("This thing is done");
        telemetry.update();

        sleep(1000);
    }

    private void dispense()
    {
        step("Dispense");
        crunchy.leftOutput.setPosition(crunchy.LEFT_OUTPUT_UP);
        crunchy.rightOutput.setPosition(crunchy.RIGHT_OUTPUT_UP);
        sleep(DISPENSE_WAIT_TIME);
        crunchy.leftOutput.setPosition(crunchy.LEFT_OUTPUT_DOWN);
        crunchy.rightOutput.setPosition(crunchy.RIGHT_OUTPUT_DOWN);
    }

    @Override
    public void stopOpMode()
    {
        crunchy.stopAndResetEncoders();
    }

    private SamplePosition detectSample(VisionTracking tracking)
    {
        runtime.reset();
        int mineralPosition = 0;

        while (ensureOpModeIsActive() && (mineralPosition == 0 || runtime.seconds() < 2))
        {
            telemetry.addLine("Looping " + getRuntime());
            mineralPosition = tracking.detectMineral();
            telemetry.update();
        }

        telemetry.clearAll();

        switch (mineralPosition)
        {
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

    private void intake(boolean on)
    {
        crunchy.intake.setPower(on ? 1 : 0);

        double intakeLiftUp = 1;
        double intakeLiftDown = -0.4;

        int intakeUpperLimit = 1500;
        int intakeLowerLimit = 0;

        while(ensureOpModeIsActive())
        {
            if ((on) && (crunchy.intakeLift.getCurrentPosition() < intakeUpperLimit))
            {
                crunchy.intakeLift.setPower(intakeLiftUp);
            } else if ((!on) && (crunchy.intakeLift.getCurrentPosition() > intakeLowerLimit))
            {
                crunchy.intakeLift.setPower(intakeLiftDown);
            } else
            {
                crunchy.intakeLift.setPower(0);
                return;
            }
        }
    }


    //Movement methods
    private void deploy()
    {
        int liftUpperLimit = 3889;

        telemetry.addLine("Deploying");
        telemetry.update();

        crunchy.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        crunchy.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int targetPos = crunchy.liftLeft.getCurrentPosition() + liftUpperLimit;

        //lower down then stopAndResetEncoders
        while (crunchy.liftLeft.getCurrentPosition() < targetPos && ensureOpModeIsActive())
        {
            crunchy.liftLeft.setPower(LIFT_POWER);
            crunchy.liftRight.setPower(LIFT_POWER);
        }

        telemetry.clear();
        telemetry.addLine("DEPLOYING HAS CEASED");
        telemetry.update();

        crunchy.liftLeft.setPower(0);
        crunchy.liftRight.setPower(0);
    }
}