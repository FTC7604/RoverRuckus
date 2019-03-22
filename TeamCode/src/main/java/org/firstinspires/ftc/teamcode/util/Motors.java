package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Motors {

    public static final double humanController(double input){
        double output = 0;

        if(input > .075){
            output = .5* Math.log10(input-0.05)+1.012;
        }
        else if(input >= -.075 && input <= .075){
            output = 500*input*input*input;
        }
        else if(input < -.075){
            output = -.5* Math.log10(-input-0.05)-1.012;
        }

        return output;
    }

    //Neverest Claims that the motor can freeload 160 rpm.
    //given that the motors will in all likelihood be understress, I'm going to assume that the robot can do at most 10 rpm

    private DcMotorEx motor;
    private double acceleration;

    private double lowerLimit;
    private double upperLimit;

    private double finalEncoder;
    private double rps = 1120*2.1;

    public Motors(DcMotorEx motor, double acceleration){
        this.motor = motor;
        this.acceleration = acceleration;
    }
    public Motors(){}

    public void start(double time, double distance){

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lowerLimit = time + (4/acceleration);
        upperLimit = lowerLimit + (distance /rps);
        finalEncoder = distance + motor.getCurrentPosition();

    }

    double sigmoid(double x) {
        /*generic sigmoid function that can be used*/
        return 1/(1+Math.pow(Math.E,-x));
    }
    double sigmoidCurve(double x) {
        double lowerCurve = sigmoid(acceleration * (x - lowerLimit));
        double upperCurve = sigmoid(acceleration * (-x + upperLimit));
        return lowerCurve * upperCurve;
    }

    double integralSigmoid(double x){
        double output = 0;

        output =  Math.log(Math.pow(Math.E,acceleration*x)+Math.pow(Math.E,acceleration*upperLimit));
        output -= Math.log(Math.pow(Math.E,acceleration*x)+Math.pow(Math.E,acceleration*lowerLimit));
        output *= Math.pow(Math.E,acceleration*upperLimit);
        output /= -acceleration*(Math.pow(Math.E,acceleration*upperLimit) - Math.pow(Math.E,acceleration*lowerLimit));

        return output;
    }
    double integralSigmoidCurve(double x){
        return integralSigmoid(x) - integralSigmoid(lowerLimit - 5/acceleration);
    }

    double definateIntegral(double lowerBound, double upperBound){
        return integralSigmoidCurve(upperBound) - integralSigmoidCurve(lowerBound);
    }

    public void setSigmoidSpeed(double time){

        //add these bad bois back on once the original function starts to work correctly
        //if(definateIntegral(time, upperLimit + 5/acceleration) > (finalEncoder - motor.getCurrentPosition())/1120) upperLimit -= .1;
        //if(definateIntegral(time, upperLimit + 5/acceleration) < (finalEncoder - motor.getCurrentPosition())/1120) upperLimit += .1;

        motor.setPower(sigmoidCurve(time));
    }

    public double getSigmoidSpeed(double time){
        return sigmoidCurve(time);
    }
    public double getRemainingDistanceEncoder(double time){return finalEncoder - motor.getCurrentPosition();}
    public double getRemainingDistanceSigmoid(double time){return rps * definateIntegral(time, upperLimit + 5/acceleration);}


}