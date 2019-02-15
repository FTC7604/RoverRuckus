package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.teamcode.util.CrunchyAutonomous;
import org.firstinspires.ftc.teamcode.util.DWAILinearOpMode;
import org.firstinspires.ftc.teamcode.util.PropertiesLoader;
import org.firstinspires.ftc.teamcode.util.vision.VisionTracking;

public class Autonomous extends DWAILinearOpMode
{
    //creates the runtime
    private ElapsedTime runtime = new ElapsedTime();
    private CrunchyAutonomous crunchy;
    private VisionTracking tracking;

    private PropertiesLoader loader = new PropertiesLoader("Autonomous");
    private final long STEP_MODE = loader.getLongProperty("stepMode");
    private final double DEPLOY_LIFT_POWER = loader.getDoubleProperty("deployLiftPower");
    private final int GOLD_POSITION = loader.getIntegerProperty("goldPosition");
    private final long DETECT_TIMEOUT = loader.getLongProperty("detectTimeout");
    private final long CLOSE_DELAY = loader.getLongProperty("closeDelay");
    private final boolean TRANSITION_TO_TELEOP = loader.getBooleanProperty("transitionToTeleop");

    private final double RELEASE_AND_DRIVE_POWER = loader.getDoubleProperty("releaseAndDrivePower");
    private final int RELEASE_AND_DRIVE_DISTANCE = loader.getIntegerProperty("releaseAndDriveDistance");
    private final double ROTATE_TO_BALL_POWER = loader.getDoubleProperty("rotateToBallPower");
    private final double ROTATE_TO_BALL_PRECISION = loader.getDoubleProperty("rotateToBallPrecision");
    private final double ROTATE_TO_BALL_ANGLE = loader.getDoubleProperty("rotateToBallAngle");
    private final double STRAFE_TO_BALL_POWER = loader.getDoubleProperty("strafeToBallPower");
    private final int STRAFE_TO_BALL_DISTANCE = loader.getIntegerProperty("strafeToBallDistance");

    private final long LOWER_ARM_WAIT_TIME = loader.getLongProperty("lowerArmWaitTime");

    private final double LEFT_BRANCH_DRIVE_TO_BALL_POWER = loader.getDoubleProperty("leftBranchDriveToBallPower");
    private final int LEFT_BRANCH_DRIVE_TO_BALL_DISTANCE = loader.getIntegerProperty("leftBranchDriveToBallDistance");
    private final double LEFT_BRANCH_HIT_BALL_POWER = loader.getDoubleProperty("leftBranchHitBallPower");
    private final int LEFT_BRANCH_HIT_BALL_DISTANCE = loader.getIntegerProperty("leftBranchHitBallDistance");
    private final double LEFT_BRANCH_STRAFE_FROM_BALL_POWER = loader.getDoubleProperty("leftBranchStrafeFromBallPower");
    private final int LEFT_BRANCH_STRAFE_FROM_BALL_DISTANCE = loader.getIntegerProperty("leftBranchStrafeFromBallDistance");
    private final double LEFT_BRANCH_DRIVE_TOWARDS_WALL_POWER = loader.getDoubleProperty("leftBranchDriveTowardsWallPower");
    private final int LEFT_BRANCH_DRIVE_TOWARDS_WALL_DISTANCE = loader.getIntegerProperty("leftBranchDriveTowardsWallDistance");

    private final double CENTER_BRANCH_HIT_BALL_POWER = loader.getDoubleProperty("centerBranchHitBallPower");
    private final int CENTER_BRANCH_HIT_BALL_DISTANCE = loader.getIntegerProperty("centerBranchHitBallDistance");
    private final double CENTER_BRANCH_STRAFE_FROM_BALL_POWER = loader.getDoubleProperty("centerBranchStrafeFromBallPower");
    private final int CENTER_BRANCH_STRAFE_FROM_BALL_DISTANCE = loader.getIntegerProperty("centerBranchStrafeFromBallDistance");
    private final double CENTER_BRANCH_DRIVE_TOWARDS_WALL_POWER = loader.getDoubleProperty("centerBranchDriveTowardsWallPower");
    private final int CENTER_BRANCH_DRIVE_TOWARDS_WALL_DISTANCE = loader.getIntegerProperty("centerBranchDriveTowardsWallDistance");

    private final double RIGHT_BRANCH_DRIVE_TO_BALL_POWER = loader.getDoubleProperty("rightBranchDriveToBallPower");
    private final int RIGHT_BRANCH_DRIVE_TO_BALL_DISTANCE = loader.getIntegerProperty("rightBranchDriveToBallDistance");
    private final double RIGHT_BRANCH_HIT_BALL_POWER = loader.getDoubleProperty("rightBranchHitBallPower");
    private final int RIGHT_BRANCH_HIT_BALL_DISTANCE = loader.getIntegerProperty("rightBranchHitBallDistance");
    private final double RIGHT_BRANCH_STRAFE_FROM_BALL_POWER = loader.getDoubleProperty("rightBranchStrafeFromBallPower");
    private final int RIGHT_BRANCH_STRAFE_FROM_BALL_DISTANCE = loader.getIntegerProperty("rightBranchStrafeFromBallDistance");
    private final double RIGHT_BRANCH_DRIVE_TOWARDS_WALL_POWER = loader.getDoubleProperty("rightBranchDriveTowardsWallPower");
    private final int RIGHT_BRANCH_DRIVE_TOWARDS_WALL_DISTANCE = loader.getIntegerProperty("rightBranchDriveTowardsWallDistance");

    private final double CRATER_POSITION_TURN_TOWARDS_DEPOT_POWER = loader.getDoubleProperty("craterPositionTurnTowardsDepotPower");
    private final double CRATER_POSITION_TURN_TOWARDS_DEPOT_ANGLE = loader.getDoubleProperty("craterPositionTurnTowardsDepotAngle");
    private final double DEPOT_POSITION_TURN_TOWARDS_DEPOT_POWER = loader.getDoubleProperty("depotPositionTurnTowardsDepotPower");
    private final double DEPOT_POSITION_TURN_TOWARDS_DEPOT_ANGLE = loader.getDoubleProperty("depotPositionTurnTowardsDepotAngle");
    private final double TURN_TOWARDS_DEPOT_PRECISION = loader.getDoubleProperty("turnTowardsDepotPrecision");
    private final double STRAFE_INTO_WALL_POWER = loader.getDoubleProperty("strafeIntoWallPower");
    private final long CRATER_POSITION_STRAFE_INTO_WALL_TIME = loader.getLongProperty("craterPositionStrafeIntoWallTime");
    private final long DEPOT_POSITION_STRAFE_INTO_WALL_TIME = loader.getLongProperty("depotPositionStrafeIntoWallTime");
    private final double STRAFE_AWAY_FROM_WALL_POWER = loader.getDoubleProperty("strafeAwayFromWallPower");
    private final int STRAFE_AWAY_FROM_WALL_DISTANCE = loader.getIntegerProperty("strafeAwayFromWallDistance");
    private final double DRIVE_TOWARDS_DEPOT_POWER = loader.getDoubleProperty("driveTowardsDepotPower");
    private final int CRATER_POSITION_DRIVE_TOWARDS_DEPOT_DISTANCE = loader.getIntegerProperty("craterPositionDriveTowardsDepotDistance");
    private final int DEPOT_POSITION_DRIVE_TOWARDS_DEPOT_DISTANCE = loader.getIntegerProperty("depotPositionDriveTowardsDepotDistance");
    private final double LOWER_LIFT_POWER = loader.getDoubleProperty("lowerLiftPower");
    private final long DISPENSE_WAIT_TIME = loader.getLongProperty("dispenseWaitTime");
    private final double DRIVE_TOWARDS_CRATER_POWER = loader.getDoubleProperty("driveTowardsCraterPower");
    private final int DRIVE_TOWARDS_CRATER_DISTANCE = loader.getIntegerProperty("driveTowardsCraterDistance");

    private enum SamplePosition
    {
        LEFT, CENTER, RIGHT
    }

    private enum IntakeLiftPosition
    {
        UP, DOWN
    }

    private boolean isCraterPosition;

    public Autonomous(boolean isCraterPosition)
    {
        this.isCraterPosition = isCraterPosition;
    }

    @Override
    public void initOpMode()
    {
        telemetry.addData("Status", "Not init-ing");
        telemetry.update();

        //Setting up all processes
        telemetry.addData("Status", "Creating Crunchy...");
        telemetry.update();
        crunchy = new CrunchyAutonomous(this);

        //resets all the encoders
        telemetry.addData("Status", "Resetting encoders.");
        telemetry.update();
        crunchy.intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Resetting encoders..");
        telemetry.update();
        crunchy.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Resetting encoders...");
        telemetry.update();
        crunchy.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Setting up encoders.");
        telemetry.update();
        crunchy.intakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Setting up encoders..");
        telemetry.update();
        crunchy.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Setting up encoders...");
        telemetry.update();
        crunchy.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Creating vision tracking...");
        telemetry.update();
        tracking = new VisionTracking(this);
        telemetry.addData("Status", "Initializing vision tracking...");
        telemetry.update();
        tracking.init();

        telemetry.addData("Status", "Closing phone...");
        telemetry.update();
        crunchy.phoneMount.setPosition(crunchy.CLOSED_PHONE);
        telemetry.addData("Status", "Initializing tensor...");
        telemetry.update();
        tracking.initTfod();

        setStepMode(STEP_MODE);

        telemetry.addData("Status", "You can press start now!");
        telemetry.update();

        if (TRANSITION_TO_TELEOP)
        {
            AutoTransitioner.transitionOnStop(this, "RR Full Teleop");
        }
    }

    @Override
    public void mainOpMode()
    {
        runtime.reset();
        telemetry.clearAll();

        step("Image detection");

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
                mineralPosition = getSample();
                break;
        }

        step(String.format("Detected %s - Deploy", mineralPosition.name()));
        deploy();

        step("Release and drive");
        crunchy.hook.setPosition(crunchy.HOOK_OPEN);
        crunchy.driveForwardForDistance(RELEASE_AND_DRIVE_POWER, RELEASE_AND_DRIVE_DISTANCE);

        step("Rotate to ball");
        crunchy.turnDegrees(ROTATE_TO_BALL_POWER, ROTATE_TO_BALL_ANGLE, ROTATE_TO_BALL_PRECISION);

        step("Strafe to ball");
        crunchy.strafeRightForDistance(STRAFE_TO_BALL_POWER, STRAFE_TO_BALL_DISTANCE);

        switch (mineralPosition)
        {
            case LEFT:
                step("[Left Branch] Drive to ball");
                crunchy.driveForwardForDistance(LEFT_BRANCH_DRIVE_TO_BALL_POWER, LEFT_BRANCH_DRIVE_TO_BALL_DISTANCE);
                crunchy.sampleArm.setPosition(crunchy.SAMPLE_ARM_DOWN);
                sleep(LOWER_ARM_WAIT_TIME);
                step("[Left Branch] Hit ball");
                crunchy.driveForwardForDistance(LEFT_BRANCH_HIT_BALL_POWER, LEFT_BRANCH_HIT_BALL_DISTANCE);
                crunchy.sampleArm.setPosition(crunchy.SAMPLE_ARM_UP);
                step("[Left Branch] Strafe away from ball");
                crunchy.strafeRightForDistance(LEFT_BRANCH_STRAFE_FROM_BALL_POWER, LEFT_BRANCH_STRAFE_FROM_BALL_DISTANCE);
                step("[Left Branch] Drive towards wall");
                crunchy.driveForwardForDistance(LEFT_BRANCH_DRIVE_TOWARDS_WALL_POWER, LEFT_BRANCH_DRIVE_TOWARDS_WALL_DISTANCE);
                break;
            case CENTER:
                crunchy.sampleArm.setPosition(crunchy.SAMPLE_ARM_DOWN);
                sleep(LOWER_ARM_WAIT_TIME);
                step("[Center Branch] Hit ball");
                crunchy.driveForwardForDistance(CENTER_BRANCH_HIT_BALL_POWER, CENTER_BRANCH_HIT_BALL_DISTANCE);
                crunchy.sampleArm.setPosition(crunchy.SAMPLE_ARM_UP);
                step("[Center Branch] Strafe away from ball");
                crunchy.strafeRightForDistance(CENTER_BRANCH_STRAFE_FROM_BALL_POWER, CENTER_BRANCH_STRAFE_FROM_BALL_DISTANCE);
                step("[Center Branch] Drive towards wall");
                crunchy.driveForwardForDistance(CENTER_BRANCH_DRIVE_TOWARDS_WALL_POWER, CENTER_BRANCH_DRIVE_TOWARDS_WALL_DISTANCE);
                break;
            case RIGHT:
                step("[Right Branch] Drive to ball");
                crunchy.driveForwardForDistance(RIGHT_BRANCH_DRIVE_TO_BALL_POWER, RIGHT_BRANCH_DRIVE_TO_BALL_DISTANCE);
                crunchy.sampleArm.setPosition(crunchy.SAMPLE_ARM_DOWN);
                sleep(LOWER_ARM_WAIT_TIME);
                step("[Right Branch] Hit ball");
                crunchy.driveForwardForDistance(RIGHT_BRANCH_HIT_BALL_POWER, RIGHT_BRANCH_HIT_BALL_DISTANCE);
                crunchy.sampleArm.setPosition(crunchy.SAMPLE_ARM_UP);
                step("[Right Branch] Strafe away from ball");
                crunchy.strafeRightForDistance(RIGHT_BRANCH_STRAFE_FROM_BALL_POWER, RIGHT_BRANCH_STRAFE_FROM_BALL_DISTANCE);
                step("[Right Branch] Drive towards wall");
                crunchy.driveForwardForDistance(RIGHT_BRANCH_DRIVE_TOWARDS_WALL_POWER, RIGHT_BRANCH_DRIVE_TOWARDS_WALL_DISTANCE);
                break;
        }

        if (isCraterPosition)
        {
            step("[Crater Position] Turn towards depot");
            crunchy.turnDegrees(CRATER_POSITION_TURN_TOWARDS_DEPOT_POWER, CRATER_POSITION_TURN_TOWARDS_DEPOT_ANGLE, TURN_TOWARDS_DEPOT_PRECISION);
            step("[Crater Position] Strafe into wall");
            crunchy.strafeRightForTime(STRAFE_INTO_WALL_POWER, CRATER_POSITION_STRAFE_INTO_WALL_TIME);
        } else
        {
            step("[Depot Position] Turn towards depot");
            crunchy.turnDegrees(DEPOT_POSITION_TURN_TOWARDS_DEPOT_POWER, DEPOT_POSITION_TURN_TOWARDS_DEPOT_ANGLE, TURN_TOWARDS_DEPOT_PRECISION);
            step("[Depot Position] Strafe into wall");
            crunchy.strafeRightForTime(STRAFE_INTO_WALL_POWER, DEPOT_POSITION_STRAFE_INTO_WALL_TIME);
        }

        step("Strafe away from wall");
        crunchy.strafeRightForDistance(STRAFE_AWAY_FROM_WALL_POWER, (isCraterPosition ? 1 : -1) * STRAFE_AWAY_FROM_WALL_DISTANCE);

        launchThread(new Runnable()
        {
            @Override
            public void run()
            {
                setIntakeLiftPosition(IntakeLiftPosition.DOWN);
                lowerLift();
            }
        });

        if (isCraterPosition)
        {
            step("[Crater Position] Drive towards depot");
            crunchy.driveForwardForDistance(DRIVE_TOWARDS_DEPOT_POWER, CRATER_POSITION_DRIVE_TOWARDS_DEPOT_DISTANCE);
        } else
        {
            step("[Depot Position] Drive towards depot");
            crunchy.driveForwardForDistance(DRIVE_TOWARDS_DEPOT_POWER, DEPOT_POSITION_DRIVE_TOWARDS_DEPOT_DISTANCE);
        }
        step("Dispense");
        crunchy.marker.setPosition(crunchy.MARKER_OPEN);
//        dispense();

        step("Drive towards crater");
        crunchy.driveForwardForDistance(DRIVE_TOWARDS_CRATER_POWER, DRIVE_TOWARDS_CRATER_DISTANCE);
        telemetry.addLine("This thing is done");
        telemetry.update();
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

        while (ensureOpModeIsActive() && (mineralPosition == 0 || runtime.seconds() < 1))
        {
            telemetry.addLine("Looping " + getRuntime());
            mineralPosition = tracking.detectMineral();
            telemetry.update();

            if(runtime.seconds() >= DETECT_TIMEOUT)
            {
                break;
            }
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
                telemetry.addLine("Detection timed out");
                return null;
        }
    }

    private void setIntakeLiftPosition(IntakeLiftPosition position)
    {
        double intakeLiftUp = 1;//power level for going up
        double intakeLiftDown = -0.6;//for going down
        // Old value -0.4

        int intakeUpperLimit = 0;//for the robot intake all the way in this is how the game starts
        int intakeLowerLimit = -1800;

        boolean intakeTargetIsUp = position == IntakeLiftPosition.UP;

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

    //Movement methods
    private void deploy()
    {
        int liftUpperLimit = 3889;

        telemetry.addLine("Deploying");
        telemetry.update();

        int targetPos = crunchy.liftLeft.getCurrentPosition() + liftUpperLimit;

        //lower down then stopAndResetEncoders
        while (crunchy.liftLeft.getCurrentPosition() < targetPos && ensureOpModeIsActive())
        {
            crunchy.liftLeft.setPower(DEPLOY_LIFT_POWER);
            crunchy.liftRight.setPower(DEPLOY_LIFT_POWER);
        }

        telemetry.clear();
        telemetry.addLine("DEPLOYING HAS CEASED");
        telemetry.update();

        crunchy.liftLeft.setPower(0);
        crunchy.liftRight.setPower(0);
    }

    private void lowerLift()
    {
        int liftUpperLimit = 3889;
        int liftLowerLimit = 75;

        int targetPos = crunchy.liftLeft.getCurrentPosition() - liftUpperLimit + liftLowerLimit;

        //lower down then stopAndResetEncoders
        while (crunchy.liftLeft.getCurrentPosition() > targetPos && ensureOpModeIsActive())
        {
            crunchy.liftLeft.setPower(LOWER_LIFT_POWER);
            crunchy.liftRight.setPower(LOWER_LIFT_POWER);
        }

        crunchy.liftLeft.setPower(0);
        crunchy.liftRight.setPower(0);
    }

    public SamplePosition getSample(){
        SamplePosition mineralPosition;
        int numLeft = 0;
        int numRight = 0;
        int numCenter = 0;

        for(int i = 0; i < 3; i++)
        {
            crunchy.phoneMount.setPosition(crunchy.OPEN_PHONE);
            mineralPosition = detectSample(tracking);
            crunchy.phoneMount.setPosition(crunchy.CLOSED_PHONE);
            sleep(CLOSE_DELAY);

            if(mineralPosition != null)
            {
                switch (mineralPosition)
                {
                    case LEFT:
                        numLeft++;
                        break;
                    case RIGHT:
                        numRight++;
                        break;
                    case CENTER:
                        numCenter++;
                        break;
                }
            }
        }

        if(numLeft > numCenter && numLeft > numRight){
            mineralPosition = SamplePosition.LEFT;
        } else if(numRight > numLeft && numRight > numCenter){
            mineralPosition = SamplePosition.RIGHT;
        } else if(numCenter > numLeft && numCenter > numRight){
            mineralPosition = SamplePosition.CENTER;
        } else{
            mineralPosition = SamplePosition.LEFT;
            telemetry.addLine("Tensor done fucked");
        }

        telemetry.addData("Sample position", mineralPosition.name());
        telemetry.update();
        tracking.shutdownTfod();

        return mineralPosition;
    }
}