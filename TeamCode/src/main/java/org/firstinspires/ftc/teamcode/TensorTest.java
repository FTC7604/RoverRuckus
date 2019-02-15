package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tensor Test", group = "Linear Op")
@Disabled
public class TensorTest extends Autonomous{
    public TensorTest(){super(true);}

    @Override
    public void mainOpMode() {
        getSample();
        sleep(3000);
    }
}