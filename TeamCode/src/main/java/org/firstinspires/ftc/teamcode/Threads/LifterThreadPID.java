package org.firstinspires.ftc.teamcode.Threads;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.Misc.LifterMethods;
import org.openftc.revextensions2.ExpansionHubMotor;

public class LifterThreadPID implements Runnable {

    public static volatile boolean kill = false;
    private long lastMillis = 0;
    private volatile int currentTicks = 0;
    private volatile int lastTicks = 0;

    public static volatile boolean finished = true;

    private ExpansionHubMotor rightLifter;
    private ExpansionHubMotor leftLifter;

    public LifterThreadPID(ExpansionHubMotor leftLifter, ExpansionHubMotor rightLifter) {
        this.leftLifter = leftLifter;
        this.rightLifter = rightLifter;
    }

    @Override
    public void run() {
        while (!kill) {
            //never run too fast
            if (SystemClock.uptimeMillis() - lastMillis < 100) {
                continue;
            }

            //set the last send time
            lastMillis = SystemClock.uptimeMillis();

            if (currentTicks != lastTicks) //we can move the motor to the new value
            {
                int lifterTolerance = 50;

                if(currentTicks > lastTicks) { //only if we go upwards
                    if (LifterMethods.getStateFromTicks(currentTicks) == LifterMethods.LIFTER.SIXTH)
                        lifterTolerance = 120;
                    else if (LifterMethods.getStateFromTicks(currentTicks) == LifterMethods.LIFTER.SEVENTH)
                        lifterTolerance = 135;
                    else if (LifterMethods.getStateFromTicks(currentTicks) == LifterMethods.LIFTER.EIGHTH)
                        lifterTolerance = 170;
                }

                finished = false;
                leftLifter.setTargetPosition(currentTicks);
                leftLifter.setTargetPositionTolerance(lifterTolerance);
                leftLifter.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);

                if (leftLifter.getCurrentPosition() < currentTicks) {
                    leftLifter.setPower(1);
                    rightLifter.setPower(1);
                    while (leftLifter.getCurrentPosition() < currentTicks && !finished) {

                    }
                } else {
                    leftLifter.setPower(-1);
                    rightLifter.setPower(-1);
                    while (leftLifter.getCurrentPosition() > currentTicks && !finished) {
                    }
                }
                leftLifter.setPower(0);
                rightLifter.setPower(0);
                finished = true;
                leftLifter.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
            }

            lastTicks = currentTicks;
        }
    }

    public void setTicks(int ticks) {
        currentTicks = ticks;
    }
}
