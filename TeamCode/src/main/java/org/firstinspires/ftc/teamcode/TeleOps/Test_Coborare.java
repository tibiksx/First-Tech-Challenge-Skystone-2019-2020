package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.Encoder;
import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Misc.UDP_Unicast_Server;

import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test Coborare Lifter", group = "Pushbot")
public class Test_Coborare extends Robot {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        robot.lifter.setPower(gamepad2.right_stick_y);
    }

    @Override
    public void stop() {
        super.stop();
    }
}