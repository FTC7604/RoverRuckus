package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;
import java.util.Map;

public class MotorControl
{
    private Map<DcMotor, MotorControlParameters> parameterMap = new HashMap<>();

    public void init(final DcMotor motor, final int targetPosition)
    {
        parameterMap.put(motor, new MotorControlParameters()
        {{
            target = motor.getCurrentPosition() + targetPosition;
            direction = targetPosition > 0 ? Direction.POSITIVE : (targetPosition == 0 ? Direction.ZERO : Direction.NEGATIVE);
        }});
    }

    public boolean isCompleted(final DcMotor motor)
    {
        MotorControlParameters params = parameterMap.get(motor);
        switch (params.direction)
        {
            case POSITIVE:
                return motor.getCurrentPosition() >= params.target;
            case NEGATIVE:
                return motor.getCurrentPosition() <= params.target;
            case ZERO:
                return true;
            default:
                throw new IllegalStateException("Unhandled switch case: " + params.direction);
        }
    }

    private static class MotorControlParameters
    {
        int target;
        Direction direction;

        public enum Direction
        {
            POSITIVE, NEGATIVE, ZERO;
        }
    }
}
