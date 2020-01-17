package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Misc.Robot;

/**
 * This class just includes the driving part of the robot, including
 * displaying the encoder values to telemetry.
 */

@TeleOp(name = "Driving + Encoders", group = "Pushbot")
public class DrivingTeleOp extends Robot {

    Hardware robot = new Hardware();

    private Telemetry.Item leftEncoderTelemetry = null;
    private Telemetry.Item rightEncoderTelemetry = null;
    private Telemetry.Item backEncoderTelemetry = null;
    private Telemetry.Item powerCoeffTelemetry = null;

    private double powerCoeff = 1;

    @Override
    public void init() {
        super.init();

        robot.init(hardwareMap);

        leftEncoderTelemetry = telemetry.addData("Left: ",0);
        rightEncoderTelemetry = telemetry.addData("Right: ",0 );
        backEncoderTelemetry = telemetry.addData("Back: ",0);
        powerCoeffTelemetry = telemetry.addData("Power Coeff: ",1);
    }


    @Override
    public void loop() {
        super.loop();

        if (controllerInputA.leftBumperOnce() && powerCoeff >= 0.2) {
            powerCoeff -= 0.1;
        } else if (controllerInputA.rightBumperOnce() && powerCoeff <= 1) {
            powerCoeff += 0.1;
        }

        powerCoeff = Range.clip(powerCoeff, 0.0, 1.0);

        float drive, turn, strafe;

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

        robot.frontLeftWheel.setPower(powerCoeff * leftFrontPower);
        robot.frontRightWheel.setPower(powerCoeff * rightFrontPower);
        robot.backLeftWheel.setPower(powerCoeff * leftBackPower);
        robot.backRightWheel.setPower(powerCoeff * rightBackPower);

        leftEncoderTelemetry.setValue(leftEncoder.getPosition());
        rightEncoderTelemetry.setValue(robot.rightEncoderMotor.getCurrentPosition());
        backEncoderTelemetry.setValue(backEncoder.getPosition());
        powerCoeffTelemetry.setValue("%.3f",powerCoeff);
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }

}
