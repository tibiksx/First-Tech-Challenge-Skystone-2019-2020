package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Misc.Robot;

public class OdometrySampleOP extends Robot {

    final double COUNTS_PER_INCH = 100;

    GlobalPositionThread globalPositionThread;
    Thread positionThread;
    @Override
    public void init() {
        super.init();

        globalPositionThread = new GlobalPositionThread(robot.backEncoderMotor,robot.leftEncoderMotor
                ,robot.rightEncoderMotor,COUNTS_PER_INCH);
        positionThread = new Thread(globalPositionThread);
        positionThread.start();

        globalPositionThread.reverseRightEncoder();
        globalPositionThread.reverseBackEncoder();
    }

    @Override
    public void loop() {
        super.loop();

        float drive, turn, strafe;

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        double leftFrontPower = (Range.clip(drive + turn - strafe, -1.0, 1.0));
        double leftBackPower = (Range.clip(drive + turn + strafe, -1.0, 1.0));
        double rightFrontPower = (Range.clip(drive - turn + strafe, -1.0, 1.0));
        double rightBackPower = (Range.clip(drive - turn - strafe, -1.0, 1.0));

        robot.frontLeftWheel.setPower(leftFrontPower);
        robot.frontRightWheel.setPower(rightFrontPower);
        robot.backLeftWheel.setPower(leftBackPower);
        robot.backRightWheel.setPower(rightBackPower);

        telemetry.addData("X position:  ",globalPositionThread.getGlobalX());
        telemetry.addData("Y position:  ",globalPositionThread.getGlobalY());
        telemetry.addData("Signed Angle:  ",globalPositionThread.getAngleInDeg());

        telemetry.addData("Left Encoder Position:  ",leftEncoder.getPosition());
        telemetry.addData("Right Encoder Position:  ",rightEncoder.getPosition());
        telemetry.addData("Back Encoder Position  :",backEncoder.getPosition());

        telemetry.update();



    }

    @Override
    public void stop() {
        super.stop();
        GlobalPositionThread.kill = true;
    }
}
