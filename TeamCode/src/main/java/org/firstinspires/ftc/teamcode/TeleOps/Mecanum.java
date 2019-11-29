package org.firstinspires.ftc.teamcode.TeleOps;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.Encoder;
import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Misc.UDP_Unicast_Server;
import org.firstinspires.ftc.teamcode.Threads.MotorThread;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.net.Inet4Address;
import java.util.HashMap;

import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test_TeleOp", group = "Pushbot")
public class Mecanum extends Robot {

    final public float powerFraction = (float) 0.6;

    double posFound1 = 0.0, posFound2 = 0.0;

    public UDP_Unicast_Server udpServer = null;

    public Encoder rightEncoder = null;
    public Encoder leftEncoder = null;
    public Encoder backEncoder = null;

    private final int ticksPerRev = 1600;
    private final int wheelDiameter = 10; //in cm


    public enum LIFTER {
        LOW,
        FIRST,
        SECOND,
        THIRD,
        FOURTH
    }

    HashMap<LIFTER, Integer> lifterStates = new HashMap<>();

    void initMap() {
        lifterStates.put(LIFTER.LOW, 0);
        lifterStates.put(LIFTER.FIRST, 1400);
        lifterStates.put(LIFTER.SECOND, 3000);
        lifterStates.put(LIFTER.THIRD, 4800);
        lifterStates.put(LIFTER.FOURTH, 6300);
    }

    public LIFTER currentState;

    @Override
    public void init() {
        super.init();
        if (usingDebugger) udpServer = new UDP_Unicast_Server(50000);
        rightEncoder = new Encoder(robot.rightEncoderMotor, ticksPerRev);
        currentState = LIFTER.LOW;
        initMap();
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
        float drive, turn, strafe;

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        double leftFrontPower = //parsePower
                (Range.clip(drive + turn - strafe, -1.0, 1.0));
        double leftBackPower = //parsePower
                (Range.clip(drive + turn + strafe, -1.0, 1.0));
        double rightFrontPower = //parsePower
                (Range.clip(drive - turn + strafe, -1.0, 1.0));
        double rightBackPower = //parsePower
                (Range.clip(drive - turn - strafe, -1.0, 1.0));

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

        //robot.lifter.setPower(gamepad2.right_stick_y);

        if (controllerInputB.dpadUpOnce()) {
            LIFTER nextState = getNextState(currentState);
            Thread runner = new Thread(new MotorThread(robot.lifter, getTicksFromState(nextState), 1));
            runner.start();
            currentState = nextState;

        }

        if (controllerInputB.dpadDownOnce()) {
            LIFTER previousState = getPreviousState(currentState);
            Thread runner = new Thread(new MotorThread(robot.lifter, getTicksFromState(previousState), 1));
            runner.start();
            currentState = previousState;
        }

        telemetry.addData("currentState:", currentState);
        telemetry.update();


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


    LIFTER getNextState(LIFTER currentState) {
        switch (currentState) {
            case LOW:
                return LIFTER.FIRST;
            case FIRST:
                return LIFTER.SECOND;
            case SECOND:
                return LIFTER.THIRD;
            case THIRD:
                return LIFTER.FOURTH;
        }
        return currentState;
    }

    LIFTER getPreviousState(LIFTER currentState) {
        switch (currentState) {
            case FIRST:
                return LIFTER.LOW;
            case SECOND:
                return LIFTER.FIRST;
            case THIRD:
                return LIFTER.SECOND;
            case FOURTH:
                return LIFTER.THIRD;
        }
        return currentState;
    }

    int getTicksFromState(LIFTER currentState) {
        switch (currentState) {
            case LOW:
                return 0;
            case FIRST:
                return 1400;
            case SECOND:
                return 3000;
            case THIRD:
                return 4800;
            case FOURTH:
                return 6300;
        }
        return -1; //code should never reach here. Yes, this function sucks...
    }

}