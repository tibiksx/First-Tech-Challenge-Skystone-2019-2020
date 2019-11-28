
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Pushbot: Teleop", group="Pushbot")

public class PushbotTeleop extends OpMode {

    // Created a hardware object, named "robot"
    Hardware robot = new Hardware();

    double posFound1 = 0.0, posFound2 = 0.0;
    double coeff = 1.0;

    // When driver hits init, execute ONCE
    @Override
    public void init() {

        telemetry.log().clear();
        telemetry.addData("Currently in:", "INIT");
        telemetry.update();

        robot.init(hardwareMap);

    }

    // After init() function, execute in loop until op mode is started
    @Override
    public void init_loop() {
    }

    // Execute ONCE when the op mode is started
    @Override
    public void start() {

        telemetry.log().clear();
        telemetry.addData("Currently in:", "Beginning of PLAY");
        telemetry.update();

        robot.frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    // Execute until driver hits STOP
    @Override
    public void loop() {

        if (gamepad1.left_bumper && coeff >= 0.2) {
            coeff -= 0.1;
        } else if (gamepad1.right_bumper && coeff <= 1) {
            coeff += 0.1;
        }

        coeff = Range.clip(coeff, 0.0, 1.0);

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

        robot.frontLeftWheel.setPower(coeff*leftFrontPower);
        robot.frontRightWheel.setPower(coeff*rightFrontPower);
        robot.backLeftWheel.setPower(coeff*leftBackPower);
        robot.backRightWheel.setPower(coeff*rightBackPower);

        // flipers

        if (gamepad1.a) {
            robot.fliper2.setPosition(1);
        }

        if (gamepad1.b) {
            robot.fliper2.setPosition(0);
        }

        if (gamepad1.x) {
            robot.fliper1.setPosition(0.90);
        }

        if (gamepad1.y) {
            robot.fliper1.setPosition(0.0);
        }

        // move the foundation (servo positions)

        if (gamepad2.y) {
            posFound1 = 1;
            posFound2 = 0;
        }

        if (gamepad2.a) {
            posFound1 = 0.3;
            posFound2 = 0.55;
        }

        // lifter and slider

        robot.lifter.setPower(-gamepad2.right_stick_y);
        robot.slider.setPower(gamepad2.left_stick_y);

        robot.foundation1.setPosition(posFound1);
        robot.foundation2.setPosition(posFound2);

    }

    // When driver hits STOP, happens once
    @Override
    public void stop() {

        telemetry.log().clear();
        telemetry.addData("Currently in:", "STOP");
        telemetry.update();
    }
}
