package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Encoder;
import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Misc.UDP_Unicast_Server;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;

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

    public LifterThread lifterThread;


    public enum LIFTER {
        LOW,
        FIRST,
        SECOND,
        THIRD,
        FOURTH
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

        currentState = LIFTER.LOW;

        lifterThread = new LifterThread(robot.lifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();


        telemetry.setAutoClear(false);
        state = telemetry.addData("nivel:",currentState);
        lifterTicks = telemetry.addData("ticks uri:",robot.lifter.getCurrentPosition());
        threadState = telemetry.addData("stare thread",LifterThread.finished);
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
        double leftBackPower =  (Range.clip(drive + turn + strafe, -1.0, 1.0));
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
        if(newPower != oldPower)
        {
            robot.lifter.setPower(newPower);
        }
        oldPower = newPower;

        if (controllerInputB.dpadUpOnce()) {

//           LIFTER nextState = getNextState(currentState);
//           Thread runner = new Thread(new MotorThread(robot.lifter, getTicksFromState(nextState), 1));
//           runner.start();
//           currentState = nextState;

           LIFTER nextState = getNextState(currentState);
           lifterThread.setTicks(getTicksFromState(nextState));
           currentState = nextState;


        }

        if (controllerInputB.dpadDownOnce()) {
//            LIFTER previousState = getPreviousState(currentState);
//            Thread runner = new Thread(new MotorThread(robot.lifter, getTicksFromState(previousState), 1));
//            runner.start();
//            currentState = previousState;

            LIFTER previousState = getPreviousState(currentState);
            lifterThread.setTicks(getTicksFromState(previousState));
            currentState = previousState;
        }


        robot.foundation1.setPosition(posFound1);
        robot.foundation2.setPosition(posFound2);

        lifterTicks.setValue(robot.lifter.getCurrentPosition());
        threadState.setValue(LifterThread.finished);
        state.setValue(currentState);
        telemetry.update();

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





//    private class LifterThread implements Runnable {
//        public volatile boolean kill = false;
//        private long lastMillis = 0;
//        private volatile int currentTicks = 0;
//        private volatile int lastTicks = 0;
//        private volatile double power = 1;
//
//        public ExpansionHubMotor lifter;
//
//        public LifterThread(ExpansionHubMotor lifter)
//        {
//            this.lifter = lifter;
//        }
//
//        @Override
//        public void run() {
//            while(true) {
//                if (kill) {
//                    break;
//                }
//                //telemetry.addData("ticks:",currentTicks);
//                //telemetry.update();
//
//                //never run too fast
//                if (SystemClock.uptimeMillis() - lastMillis < 50) {
//                    continue;
//                }
//                //set the last send time
//                lastMillis = SystemClock.uptimeMillis();
//
//                if(currentTicks != lastTicks) //we can move the motor to the new value
//                {
//                    //telemetry.addData("its a go","go");
//                    //telemetry.update();
//                    lifter.setTargetPosition(currentTicks);
//                    lifter.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
//                    lifter.setPower(power);
//                    double startTime = SystemClock.uptimeMillis();
//                    while(lifter.getCurrentPosition() < lifter.getTargetPosition()) {
//                        if(SystemClock.uptimeMillis() - startTime > 5000) break;
//                        //telemetry.addData("encoder: ",lifter.getCurrentPosition());
//                        //telemetry.update();
//                    }
//                    lifter.setPower(0);
//                    lifter.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
//                }
//
//                lastTicks = currentTicks;
//            }
//        }
//
//        public void setTicks(int ticks){
//            this.currentTicks = ticks;
//        }
//    }

}