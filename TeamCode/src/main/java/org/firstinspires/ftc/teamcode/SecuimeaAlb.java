package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="SecuimeaAlb", group="Pushbot")
public class SecuimeaAlb extends LinearOpMode {

    Hardware robot = new Hardware();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        timer.reset();
        while(timer.seconds() < 1.5) {
            setWheelsPower(0.15, -0.15, -0.15, 0.15);
        }

        timer.reset();
        while(timer.seconds() < 1.3) {
            setWheelsPower(-0.45, -0.45, -0.45, -0.45);
        }

        timer.reset();
        while(timer.seconds() < 1.3) {
            setWheelsPower(-0.45, -0.45, -0.45, -0.45);
        }

        sleep(1000);
        robot.foundation1.setPosition(0.3);
        robot.foundation2.setPosition(0.55);
        sleep(1000);

        timer.reset();
        while(timer.seconds() < 2.8) {
            setWheelsPower(0.45, 0.45, 0.45, 0.45);
        }

        sleep(1000);
        robot.foundation1.setPosition(1);
        robot.foundation2.setPosition(0);

        timer.reset();
        while(timer.seconds() < 3) {
            setWheelsPower(-0.45, 0.45, 0.45, -0.45);
        }

    }

    public void setWheelsPower(double powerFrontLeft, double powerFrontRight, double powerBackLeft, double powerBackRight) {

        robot.frontLeftWheel.setPower(powerFrontLeft);
        robot.frontRightWheel.setPower(powerFrontRight);
        robot.backLeftWheel.setPower(powerBackLeft);
        robot.backRightWheel.setPower(powerBackRight);
    }

}
