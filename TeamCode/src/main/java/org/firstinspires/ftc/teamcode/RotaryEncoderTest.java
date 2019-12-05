package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="EncoderTest", group="Pushbot")
@Disabled
public class RotaryEncoderTest extends LinearOpMode {

    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        while(true) {
            telemetry.addData("Encoder reading:", robot.test1.getCurrentPosition() / 1600);
            telemetry.update();

            // 1 full rotation - 1600 ticks.
        }

    }
}
