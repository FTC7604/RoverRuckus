/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Motors;
import org.firstinspires.ftc.teamcode.util.PropertiesLoader;
import org.firstinspires.ftc.teamcode.util.SigmoidMotor;


@Autonomous(name="Sigmoid")
//@Disabled
public class sigmoidOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor freeMotor = null;
    private DcMotor extensionMotor = null;

    private double loop;

    private PropertiesLoader loader = new PropertiesLoader("Sigmoid");
    private final double ACCELERATION = loader.getDoubleProperty("acceleration");
    private final double DISTANCE = loader.getDoubleProperty("distance");

    //from closed, negative is out
    //all the way extended is -6230
    //all the way in is 0.
    //OUT IS NEGATIVE.

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        freeMotor = hardwareMap.get(DcMotor.class, "m");
        extensionMotor = hardwareMap.get(DcMotor.class, "e");

        freeMotor.setDirection(DcMotor.Direction.FORWARD);
        extensionMotor.setDirection(DcMotor.Direction.FORWARD);

        SigmoidMotor sigmoidTest = new SigmoidMotor(freeMotor, ACCELERATION);

        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        AutonomousStep mode = AutonomousStep.START;
        loop = 0;

        waitForStart();
        runtime.reset();

        while(mode != AutonomousStep.END && opModeIsActive()){

            switch(mode) {
                case START:
                    mode = mode.next();
                    sigmoidTest.start(DISTANCE);
                    break;
                case FREE_MOTOR_20_ROTATIONS:
                    sigmoidTest.setVelocity(time);

                    if(sigmoidTest.isDone())mode = mode.next();
                    break;
            }

            if(mode == AutonomousStep.START ) startTime = time;
            if(mode == AutonomousStep.END) endTime = time;

            loop++;

            telemetry.addData("Mode: ", mode.toString());
            telemetry.addData("Stage: ", sigmoidTest.getCurrentStage());
            telemetry.addData("Velocity:",sigmoidTest.velocity);
            telemetry.addData("Remaining Distance: ",sigmoidTest.remainingDistance());
            telemetry.addData("Proportional: ",sigmoidTest.decellerationError.getProportionalError());
            telemetry.addData("Integral: ",sigmoidTest.decellerationError.getIntegralError());
            telemetry.addData("Derivative: ",sigmoidTest.decellerationError.getDerivativeError());
            telemetry.update();
        }

        while(opModeIsActive()){
            telemetry.addData("Remaining Distance: ",sigmoidTest.remainingDistance());
            telemetry.update();
        }
    }

    //0.54984, 0.55135, and 0.54991 seconds per revolution
    //1.81870, 1.81374, and 0.81871 revolutions per second

    private double startTime;
    private double endTime;
    public enum AutonomousStep{
        START,
        FREE_MOTOR_20_ROTATIONS,
        END;

        private static AutonomousStep[] steps = values();

        public AutonomousStep next()
        {
            return steps[(this.ordinal() + 1) % steps.length];
        }
        public AutonomousStep previous() { return steps[(this.ordinal() - 1) < 0 ? steps.length - 1 : this.ordinal() - 1]; }
    }


}
