package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Misc.LifterMethods;
import org.firstinspires.ftc.teamcode.Threads.LifterThreadPID;

import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Misc.LifterMethods.LIFTER.LOW;

/**
 * Main TeleOp
 */

@TeleOp(name = "Main TeleOp", group = "Pushbot")
public class Mecanum extends Robot {

    private double powerCoeff = 1.0;

    public LifterThreadPID lifterThread;

    public LifterMethods.LIFTER currentState;

    private double lifterOldPower = 0;
    private double lifterNewPower = 0;

    private Telemetry.Item leftEncoderTelemetry= null;
    private Telemetry.Item rightEncoderTelemetry = null;
    private Telemetry.Item backEncoderTelemetry = null;

    @Override
    public void init() {
        super.init();


        currentState = LOW;
        lifterThread = new LifterThreadPID(robot.leftLifter, robot.rightLifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();

        telemetry.setAutoClear(false);
        leftEncoderTelemetry = telemetry.addData("Left: ",0);
        rightEncoderTelemetry = telemetry.addData("Right: ",0 );
        backEncoderTelemetry = telemetry.addData("Back: ",0);
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

        lifterNewPower = gamepad2.right_stick_y;
        if (lifterNewPower != lifterOldPower && LifterThreadPID.finished) {
            robot.leftLifter.setPower(-lifterNewPower);
            robot.rightLifter.setPower(-lifterNewPower);
        }
        lifterOldPower = lifterNewPower;

        if (controllerInputB.dpadUpOnce() && LifterThreadPID.finished) {
            LifterMethods.LIFTER nextState = LifterMethods.getNextState(currentState);
            lifterThread.setTicks(LifterMethods.getTicksFromState(nextState));
            currentState = nextState;
        }

        if (controllerInputB.dpadDownOnce() && LifterThreadPID.finished) {
            LifterMethods.LIFTER previousState = LifterMethods.getPreviousState(currentState);
            lifterThread.setTicks(LifterMethods.getTicksFromState(previousState));
            currentState = previousState;
        }

        leftEncoderTelemetry.setValue("%.3f",leftEncoder.getDistance());
        rightEncoderTelemetry.setValue("%.3f",rightEncoder.getDistance());
        backEncoderTelemetry.setValue("%.3f",backEncoder.getDistance());
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
    }

}