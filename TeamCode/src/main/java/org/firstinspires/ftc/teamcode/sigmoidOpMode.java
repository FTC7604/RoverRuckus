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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Motors;


@TeleOp(name="Sigmoid Opmode", group="Linear Opmode")
//@Disabled
public class sigmoidOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx freeMotor = null;
    private DcMotor extensionMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        freeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "m");
        extensionMotor = hardwareMap.get(DcMotor.class,"e");

        freeMotor.setDirection(DcMotor.Direction.FORWARD);
        extensionMotor.setDirection(DcMotor.Direction.FORWARD);

        freeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        freeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Motors sigmoidTest = new Motors(freeMotor, 2);
        loop = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            loop++;
            startTime = time;

            if(gamepad1.a) sigmoidTest.start(time, 5000);
            sigmoidTest.setSigmoidSpeed(time);

            endTime = time;

            telemetry.addData("Sigmoid Power", sigmoidTest.getSigmoidSpeed(time));
            telemetry.addData("Remaining Encoders", sigmoidTest.getRemainingDistanceEncoder(time));
            telemetry.addData("Remaining Sigmoid", sigmoidTest.getRemainingDistanceSigmoid(time));
            telemetry.addLine("");
            telemetry.addData("Time (ms)", time);
            telemetry.addData("Time of previous loop (ms)", startTime - endTime);
            telemetry.addData("Average time per loop (ms)", loop/time);
            telemetry.update();
        }
    }

    private double startTime;
    private double endTime;
    private double loop;
}
