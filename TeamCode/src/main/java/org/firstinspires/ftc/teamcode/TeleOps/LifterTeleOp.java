package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Misc.LifterMethods;
import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;

/**
 * This class includes the lifter and the extension arm
 */

@TeleOp(name = "Lifter + Extension Arm Only TeleOp", group = "Test TeleOp")
public class LifterTeleOp extends Robot {

    public LifterThread lifterThread;

    public LifterMethods.LIFTER currentState;

    private double lifterOldPower = 0;
    private double lifterNewPower = 0;

    private double extensionOldPower = 0;
    private double extensionNewPower = 0;

    private DigitalChannel lifterLimitSwitch;

    int i = 0;
    LifterMethods.LIFTER[] level = {LifterMethods.LIFTER.LOW, LifterMethods.LIFTER.FIRST, LifterMethods.LIFTER.SECOND, LifterMethods.LIFTER.THIRD, LifterMethods.LIFTER.FOURTH};

    @Override
    public void start() {
        super.start();
        lifterThread = new LifterThread(robot.leftLifter, robot.rightLifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();

        currentState = LifterMethods.LIFTER.LOW;

//        lifterLimitSwitch = hardwareMap.get(DigitalChannel.class,"LS");
//        lifterLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        super.loop();

        //-----------------EXTENSION ARM----------------------------
//        extensionNewPower = gamepad2.left_stick_x;
//        if(extensionNewPower != extensionOldPower){
//            robot.extension.setPower(extensionNewPower);
//        }
//        extensionOldPower = extensionNewPower;

        //--------------------LIFTER------------------------------------
        lifterNewPower = gamepad2.right_stick_y;
        if (lifterNewPower != lifterOldPower && LifterThread.finished) {
            robot.leftLifter.setPower(-lifterNewPower);
            robot.rightLifter.setPower(-lifterNewPower);
        }
        lifterOldPower = lifterNewPower;

//        if (controllerInputB.dpadUpOnce() && LifterThread.finished) {
//            LifterMethods.LIFTER nextState = LifterMethods.getNextState(currentState);
//            lifterThread.setTicks(LifterMethods.getTicksFromState(nextState));
//            currentState = nextState;
//        }
//
//        if (controllerInputB.dpadDownOnce() && LifterThread.finished) {
//            LifterMethods.LIFTER previousState = LifterMethods.getPreviousState(currentState);
//            lifterThread.setTicks(LifterMethods.getTicksFromState(previousState));
//            currentState = previousState;
//        }


        if (controllerInputB.dpadLeftOnce() && i > 0) i--;

        if (controllerInputB.dpadRightOnce() && i < 4) i++;

        if (controllerInputB.dpadUpOnce() && LifterThread.finished) {

            lifterThread.setTicks(LifterMethods.getTicksFromState(level[i]));
            currentState = level[i];
        }

        if (controllerInputB.dpadDownOnce() && LifterThread.finished) {

            lifterThread.setTicks(LifterMethods.getTicksFromState(level[i]));
            currentState = level[i];
        }
        telemetry.addData("Lifter State: ", currentState + "  Next:" + i +
                + robot.leftLifter.getCurrentPosition());
        //telemetry.addData("Limit switch state: ",lifterLimitSwitch.getState());
        telemetry.update();

        calculateStatefromTicks();




    }

    void calculateStatefromTicks() {
        int ticks = robot.leftLifter.getCurrentPosition();
        if (ticks < LifterMethods.getTicksFromState(LifterMethods.LIFTER.LOW)) {
            currentState = LifterMethods.LIFTER.LOW;
            return;
        }
        if (ticks > LifterMethods.getTicksFromState(LifterMethods.LIFTER.LOW)
                && ticks < LifterMethods.getTicksFromState(LifterMethods.LIFTER.SECOND)) {
            currentState = LifterMethods.LIFTER.FIRST;
            return;
        }

        if (ticks > LifterMethods.getTicksFromState(LifterMethods.LIFTER.FIRST)
                && ticks < LifterMethods.getTicksFromState(LifterMethods.LIFTER.THIRD)) {
            currentState = LifterMethods.LIFTER.SECOND;
            return;
        }

        if (ticks > LifterMethods.getTicksFromState(LifterMethods.LIFTER.SECOND)
                && ticks < LifterMethods.getTicksFromState(LifterMethods.LIFTER.FOURTH)) {
            currentState = LifterMethods.LIFTER.THIRD;
            return;
        }

        if (ticks > LifterMethods.getTicksFromState(LifterMethods.LIFTER.THIRD)
                && ticks < LifterMethods.getTicksFromState(LifterMethods.LIFTER.FIFTH)) {
            currentState = LifterMethods.LIFTER.FOURTH;
            return;
        }

        if (ticks > LifterMethods.getTicksFromState(LifterMethods.LIFTER.FOURTH)
                && ticks < LifterMethods.getTicksFromState(LifterMethods.LIFTER.SIXTH)) {
            currentState = LifterMethods.LIFTER.FIFTH;
            return;
        }

        if (ticks > LifterMethods.getTicksFromState(LifterMethods.LIFTER.FIFTH))
            currentState = LifterMethods.LIFTER.SIXTH;
    }
}
