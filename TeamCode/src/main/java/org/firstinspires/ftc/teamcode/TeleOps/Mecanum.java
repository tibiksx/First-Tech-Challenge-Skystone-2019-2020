package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Misc.UDP_Unicast_Server;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;
import org.firstinspires.ftc.teamcode.Misc.LifterMethods;

import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Test_TeleOp", group = "Pushbot")
public class Mecanum extends Robot {

    final private float powerFraction = (float) 0.6;

    private double posFound1 = 0.0, posFound2 = 0.0;

    public UDP_Unicast_Server udpServer = null;

    public LifterThread lifterThread;

    public enum LIFTER {
        LOW,
        FIRST,
        SECOND,
        THIRD,
        FOURTH,
        FIFTH,
        SIXTH
    }
    public LIFTER currentState;

    double oldPower = 0;
    double newPower = 0;

    public Telemetry.Item leftEncoderTicks = null;
    public Telemetry.Item motorVelocity = null;

    @Override
    public void init() {
        super.init();
        if (usingDebugger) udpServer = new UDP_Unicast_Server(50000);

        lifterThread = new LifterThread(robot.lifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();

        telemetry.setAutoClear(false);
        leftEncoderTicks = telemetry.addData( "encoder Left:",leftEncoder.getPosition());
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
        /* TODO Optimize code with Expansion Hub Bulk Data */
        super.loop();

        float drive, turn, strafe;

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        double leftFrontPower = (Range.clip(drive + turn - strafe, -1.0, 1.0));
        double leftBackPower = (Range.clip(drive + turn + strafe, -1.0, 1.0));
        double rightFrontPower = (Range.clip(drive - turn + strafe, -1.0, 1.0));
        double rightBackPower = (Range.clip(drive - turn - strafe, -1.0, 1.0));

        robot.frontLeftWheel.setPower(powerFraction * leftFrontPower);
        robot.frontRightWheel.setPower(powerFraction * rightFrontPower);
        robot.backLeftWheel.setPower(powerFraction * leftBackPower);
        robot.backRightWheel.setPower(powerFraction * rightBackPower);

        if (controllerInputB.AOnce()){
            posFound1 = 1;
        }

        if (controllerInputB.BOnce()) {
            posFound1 = 0.3;
        }

        if (controllerInputB.XOnce()) {
            posFound2 = 0;
        }

        if (controllerInputB.YOnce()) {
            posFound2 = 0.55;
        }

        if (gamepad1.a) {
            robot.flipper2.setPosition(1);
        }

        if (controllerInputA.AOnce()) {
            robot.flipper2.setPosition(0);
        }

        if (controllerInputA.XOnce()) {
            robot.flipper1.setPosition(0.90);
        }

        if (controllerInputA.YOnce()) {
            robot.flipper1.setPosition(0.0);
        }


        newPower = gamepad2.right_stick_y;
        if (newPower != oldPower && LifterThread.finished) {
            robot.lifter.setPower(-newPower);
        }
        oldPower = newPower;

        if (controllerInputB.dpadUpOnce() && LifterThread.finished) {
            LIFTER nextState = LifterMethods.getNextState(currentState);
            lifterThread.setTicks(LifterMethods.getTicksFromState(nextState));
            currentState = nextState;
        }

        if (controllerInputB.dpadDownOnce() && LifterThread.finished) {
            LIFTER previousState = LifterMethods.getPreviousState(currentState);
            lifterThread.setTicks(LifterMethods.getTicksFromState(previousState));
            currentState = previousState;
        }

        robot.foundation1.setPosition(posFound1);
        robot.foundation2.setPosition(posFound2);

        leftEncoderTicks.setValue(leftEncoder.getPosition());

    }

    @Override
    public void stop() {
        super.stop();
    }

}