package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.LifterMethods;
import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;

@TeleOp(name = "Lifter Only TeleOp", group = "Test TeleOp")
public class LifterTeleOp extends Robot {

    public LifterThread lifterThread;

    public LifterMethods.LIFTER currentState;

    private double oldPower = 0;
    private double newPower = 0;

    @Override
    public void start() {
        super.start();
        lifterThread = new LifterThread(robot.lifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();
    }

    @Override
    public void loop() {
        super.loop();

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
    }

}
