package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ThreadTest", group="Pushbot")
public class ThreadTest extends LinearOpMode {

    Hardware robot = new Hardware();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        telemetry.addData("imi fut mama", "mama");
        telemetry.update();

        Thread motor1 = new Thread(new RunMotor(robot.test1, 3000));
        Thread motor2 = new Thread(new RunMotor(robot.test2, 6000));

        motor1.start();
        motor2.start();

        timer.reset();
        telemetry.addData("current time: ", timer);
        telemetry.update();
        try {
            motor1.join();
            motor2.join();
        } catch (InterruptedException e) {
            telemetry.addData("Aiurea caine!", " caine :( ");
            telemetry.update();
        }

        telemetry.addData("final time: ", timer);
        telemetry.update();

        sleep(500);
    }

    private class RunMotor implements Runnable {

        DcMotor motor;
        int noOfTicks;

        RunMotor(DcMotor motor, int ticks) {
            this.motor = motor;
            this.noOfTicks = ticks;
        }

        @Override
        public void run() {

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(noOfTicks);

            motor.setPower(1.0);

            while (motor.getCurrentPosition() < noOfTicks && motor.isBusy()) {
                telemetry.addData("encd", motor.getCurrentPosition());
                telemetry.update();
            }

            motor.setPower(0.0);

            telemetry.addData("Putere curenta:", motor.getPower());
            telemetry.update();

            sleep(200);
        }
    }
}
