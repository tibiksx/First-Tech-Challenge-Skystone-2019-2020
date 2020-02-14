package org.firstinspires.ftc.teamcode.Threads;

import android.os.SystemClock;

import org.openftc.revextensions2.ExpansionHubMotor;

/**
 * Thread running in background which controls the lifter. It is very useful
 * as it allows for multiple commands at the same time.
 */

public class LifterThread implements Runnable {
    public static volatile boolean kill = false;
    private long lastMillis = 0;
    private volatile int currentTicks = 0;
    private volatile int lastTicks = 0;

    public static volatile boolean finished = true;

    public ExpansionHubMotor leftLifter;
    public ExpansionHubMotor rightLifter;

    public LifterThread(ExpansionHubMotor leftLifter, ExpansionHubMotor rightLifter) {
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
                finished = false;
                double startTime = SystemClock.uptimeMillis();

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

                finished = true;
                leftLifter.setPower(0);
                rightLifter.setPower(0);
            }

            lastTicks = currentTicks;
        }
    }

    public void setTicks(int ticks) {
        this.currentTicks = ticks;
    }
}
