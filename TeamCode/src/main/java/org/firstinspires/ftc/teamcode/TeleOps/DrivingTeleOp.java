package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Misc.Robot;

@TeleOp(name="Tele Op Driving Only",group = "Test TeleOp")
public class DrivingTeleOp extends Robot {

    private double powerCoeff = 1.0;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();

        if (gamepad1.left_bumper && powerCoeff >= 0.2) {
            powerCoeff -= 0.1;
        } else if (gamepad1.right_bumper && powerCoeff <= 1) {
            powerCoeff += 0.1;
        }

        powerCoeff = Range.clip(powerCoeff, 0.0, 1.0);

        float drive, turn, strafe;

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        // clip() = demands a number to be in certain bounds
        // number is calculated and then processed
        double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

        robot.frontLeftWheel.setPower(powerCoeff * leftFrontPower);
        robot.frontRightWheel.setPower(powerCoeff * rightFrontPower);
        robot.backLeftWheel.setPower(powerCoeff * leftBackPower);
        robot.backRightWheel.setPower(powerCoeff * rightBackPower);
    }
}
