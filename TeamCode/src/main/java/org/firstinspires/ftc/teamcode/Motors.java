package org.firstinspires.ftc.teamcode;

public class Motors {
    public static final double controller(double input){
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
    public static final double curve(double encoderLimit, double curviness, double encoderValue){
        double distanceToEncoderLimit = encoderValue - encoderLimit;
        double output = 0;

        if(distanceToEncoderLimit == 0)output = 0;
        else if(distanceToEncoderLimit > 0){
            output = encoderCurve(distanceToEncoderLimit, curviness, encoderValue);
        }
        else if(distanceToEncoderLimit < 0){
            output = encoderCurve(distanceToEncoderLimit, curviness, encoderValue);
        }
    }

    private static final double encoderCurve(double distanceToEncoderLimit, double curviness, double encoders){
        double output = 0;

        output = encoders - distanceToEncoderLimit;
        output = Math.pow(output, distanceToEncoderLimit/curviness);
        output = output/(Math.pow(distanceToEncoderLimit, (distanceToEncoderLimit/curviness)));
        output = Math.sqrt(1 - output);

        return output;
    }
}