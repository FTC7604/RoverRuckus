package org.firstinspires.ftc.teamcode;
import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//for the IMU
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/*@TeleOp(name = "Double Mecanum", group = "Linear Opmode")
//@Disabled
public class DoubleGyroMecanumDrive extends LinearOpMode{
    //class Variables
    final private double pi = 3.14159;
    
    //we want the run time to work
    private ElapsedTime runtime = new ElapsedTime();

    //creates the motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private BNO055IMU rightIMU = null;
    private BNO055IMU leftIMU = null;

    //creates the parameters that the IMU uses.
    private void startIMU() {
        // We are expecting the IMU to be attached to an I2C port on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        rightIMU.initialize(parameters);
        leftIMU.initialize(parameters);
    }

    //Calibrates the bot's IMU
    private void calibrateIMU() {
        //reads the calibration data and creates a .json file that it creates each new one
        BNO055IMU.CalibrationData rightCalibrationData = rightIMU.readCalibrationData();
        String rightString = "rightIMUcalibration.json";
        File rightfile = AppUtil.getInstance().getSettingsFile(rightString);
        ReadWriteFile.writeFile(rightfile, rightCalibrationData.serialize());

        BNO055IMU.CalibrationData leftCalibrationData = leftIMU.readCalibrationData();
        String leftString = "leftIMUcalibration.json";
        File leftfile = AppUtil.getInstance().getSettingsFile(leftString);
        ReadWriteFile.writeFile(leftfile, leftCalibrationData.serialize());
    }

    //Uses the bots IMU to calculate rotation relative to the z-axis
    private double getYaw(){

        //angle that will be returned
        double bot_angle = 0;

        //unfortuently, to the best of my knowledge
        bot_angle = (leftIMU.getAngularOrientation().firstAngle + rightIMU.getAngularOrientation().firstAngle)/2;

        bot_angle = calcAngle(bot_angle);
        return bot_angle;
    }

    //Uses the bots IMU to calculate rotation relative to the y-axis - straight forward is positive y
    private double getRoll(){

        //angle that will be returned
        double bot_angle = 0;

        //unfortuently, to the best of my knowledge
        bot_angle = (leftIMU.getAngularOrientation().secondAngle + rightIMU.getAngularOrientation().secondAngle)/2;

        bot_angle = calcAngle(bot_angle);
        return bot_angle;
    }

    //Uses the bots IMU to calculate rotation relative to the y-axis - straight forward is positive y
    private double getPitch(){

        //angle that will be returned
        double bot_angle = 0;

        //unfortuently, to the best of my knowledge
        bot_angle = bot_angle = (leftIMU.getAngularOrientation().thirdAngle + rightIMU.getAngularOrientation().thirdAngle)/2;

        bot_angle = calcAngle(bot_angle);
        return bot_angle;
    }

    //will convert an angle into a value that is greater than or equal to 0 and less than 2pi
    private double calcAngle(double angle){
        //greater than or equal to 0
        while(angle < 0){
            angle += 2 * pi;
        }
        //less than pi
        while(angle >= 2 * pi){
            angle -= 2 * pi;
        }
        return angle;
    }
    
    //calculates the magnitude of movement from an x imput and a y imput
    private double getControllerMagnitude(double controller_x_value, double controller_y_value){
        //x and y and the calculated x and y imputs.
        double controller_magnitude = 0;
        controller_magnitude = hypot(controller_x_value, controller_y_value);
        return controller_magnitude;
    }
    
    //calculates the angle between the vector of the controller and the x axis. 
    private double getControllerAngle(double controller_x_value, double controller_y_value, double controller_magnitude){
       double controller_angle = 0;
        if (controller_y_value >= 0){
            controller_angle = acos(controller_x_value/controller_magnitude);
        } 
        else if (controller_y_value< 0){
            controller_angle = 2 * pi - acos(controller_x_value/controller_magnitude);
        }
        return controller_angle;
    }

    //imputs the values that go to the motors on the mecanum drive train. 
    private void imputMechanumMotors(double bot_x_direction, double bot_y_direction, double bot_rotation){
        
        //sets the power for the motors on the bot.
        leftFront.setPower(bot_y_direction - bot_x_direction + bot_rotation);
        leftBack.setPower(bot_y_direction + bot_x_direction + bot_rotation);
        rightFront.setPower(bot_y_direction + bot_x_direction - bot_rotation);
        rightBack.setPower(bot_y_direction - bot_x_direction - bot_rotation);
    }
    
    //start the code that the actually run
    @Override
    public void runOpMode(){
        
        //tells us that the code has started
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        //the imu
        leftIMU = hardwareMap.get(BNO055IMU.class, "li");
        rightIMU = hardwareMap.get(BNO055IMU.class, "ri");
        
        //mecanum motors
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        
        //reverses the left motors so that all the motors are oriented the same
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        
        //set the papmeters and calibrates the IMU
        startIMU(); 
        calibrateIMU();
        
        //starts the loop that runs for a TeleOp
        waitForStart();
        runtime.reset();
        
        //Variables created from the controller imput, probably too many. 
        double controller_y_value = 0;
        double controller_x_value = 0;
        double controller_magnitude = 0;
        double controller_angle = 0;

        //Variables that will control the bot movement. 
        double bot_angle = 0;
        double bot_y_direction = 0;
        double bot_x_direction = 0;
        double bot_rotation = 0;

        //code that repeats untile the teleOp is over
        while (opModeIsActive()) {

            //so that I don't get confused
            Float y = gamepad1.left_stick_y;
            Float x = gamepad1.left_stick_x;

            //Turns the motion into a vector that can be manipulated
            controller_y_value = y * abs(y);
            controller_x_value = x * abs(x);

            //get the magnitude and angle of the controller. 
            controller_magnitude = getControllerMagnitude(controller_x_value, controller_y_value);
            controller_angle = getControllerAngle(controller_x_value, controller_y_value, controller_magnitude);

            //calculates the angle that the bot should be at. 
            bot_angle = controller_angle - getYaw();
            bot_angle = calcAngle(bot_angle);

            //translates the bot anngle and magnitude to calculate its values
            bot_y_direction = controller_magnitude / sin(bot_angle);
            bot_x_direction = controller_magnitude / cos(bot_angle);

            //Uses the trigger values to calculate rotation
            bot_rotation = gamepad1.left_trigger - gamepad1.right_trigger;

            //imputs the power into the mechanum motors
            imputMechanumMotors(bot_x_direction, bot_y_direction, bot_rotation);

            //updates the telemetry
            telemetry.addData("bot_y_direction", bot_y_direction);
            telemetry.addData("bot_x_direction", bot_x_direction);
            telemetry.addData("bot_rotation", bot_rotation);
            telemetry.addData("bot_angle", getYaw());
            telemetry.update();
        }
    }
}*/