package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Encoder;
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

    public Encoder rightEncoder = null;
    public Encoder leftEncoder = null;
    public Encoder backEncoder = null;

    private final int ticksPerRev = 1600;
    private final int wheelDiameter = 10; //in cm

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

    public Telemetry.Item state;
    public Telemetry.Item threadState;
    public Telemetry.Item lifterTicks;

    @Override
    public void init() {
        super.init();
        if (usingDebugger) udpServer = new UDP_Unicast_Server(50000);

        rightEncoder = new Encoder(robot.rightEncoderMotor, ticksPerRev);

        lifterThread = new LifterThread(robot.lifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();

        telemetry.setAutoClear(false);
        state = telemetry.addData("nivel:", currentState);
        lifterTicks = telemetry.addData("ticks uri:", robot.lifter.getCurrentPosition());
        threadState = telemetry.addData("stare thread", LifterThread.finished);
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

        if (gamepad2.a) {
            posFound1 = 1;
        }

        if (gamepad2.b) {
            posFound1 = 0.3;
        }

        if (gamepad2.x) {
            posFound2 = 0;
        }

        if (gamepad2.y) {
            posFound2 = 0.55;
        }

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


        newPower = gamepad2.right_stick_y;
        if (newPower != oldPower && LifterThread.finished) {
            robot.lifter.setPower(newPower);
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

        lifterTicks.setValue(robot.lifter.getCurrentPosition());
        threadState.setValue(LifterThread.finished);
        state.setValue(currentState);
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
    }

}