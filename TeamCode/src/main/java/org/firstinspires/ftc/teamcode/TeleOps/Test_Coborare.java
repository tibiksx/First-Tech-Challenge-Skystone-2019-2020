package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.openftc.revextensions2.ExpansionHubMotor;

@TeleOp(name = "Lifter Only TeleOP ", group = "Pushbot")
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

    double oldPower = 0;
    double newPower = 0;
    @Override
    public void loop() {
        newPower = gamepad2.right_stick_y;
        if(newPower != oldPower)
        {
            robot.lifter.setPower(newPower);
        }
        oldPower = newPower;

        PIDCoefficients pidCoefficients = robot.lifter.getPIDCoefficients(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("p:",pidCoefficients.p);
        telemetry.addData("i:",pidCoefficients.i);
        telemetry.addData("d:",pidCoefficients.d);
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}