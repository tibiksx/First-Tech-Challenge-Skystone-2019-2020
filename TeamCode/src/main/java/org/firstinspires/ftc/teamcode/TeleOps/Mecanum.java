package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;
import org.firstinspires.ftc.teamcode.Misc.LifterMethods;

import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Main TeleOp", group = "Pushbot")
public class Mecanum extends Robot {

    private double powerCoeff = 1.0;

    private double posFound1 = 0.0, posFound2 = 0.0;

    public LifterThread lifterThread;

    public LifterMethods.LIFTER currentState;

    private double oldPower = 0;
    private double newPower = 0;

    @Override
    public void init() {
        super.init();

        lifterThread = new LifterThread(robot.lifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();

        telemetry.setAutoClear(false);
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

        if (gamepad1.a) {
            robot.flipper2.setPosition(1);
        }

        if (gamepad1.b) {
            robot.flipper2.setPosition(0);
        }

        if (gamepad1.x) {
            robot.flipper1.setPosition(0.90);
        }

        if (gamepad1.y) {
            robot.flipper1.setPosition(0.0);
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
        robot.slider.setPower(gamepad2.left_stick_y);

        robot.foundation1.setPosition(posFound1);
        robot.foundation2.setPosition(posFound2);

        newPower = gamepad2.right_stick_y;
        if (newPower != oldPower && LifterThread.finished) {
            robot.lifter.setPower(-newPower);
        }
        oldPower = newPower;

        if (controllerInputB.dpadUpOnce() && LifterThread.finished) {
            LifterMethods.LIFTER nextState = LifterMethods.getNextState(currentState);
            lifterThread.setTicks(LifterMethods.getTicksFromState(nextState));
            currentState = nextState;
        }

        if (controllerInputB.dpadDownOnce() && LifterThread.finished) {
            LifterMethods.LIFTER previousState = LifterMethods.getPreviousState(currentState);
            lifterThread.setTicks(LifterMethods.getTicksFromState(previousState));
            currentState = previousState;
        }

        robot.foundation1.setPosition(posFound1);
        robot.foundation2.setPosition(posFound2);

    }

    @Override
    public void stop() {
        super.stop();
    }

}