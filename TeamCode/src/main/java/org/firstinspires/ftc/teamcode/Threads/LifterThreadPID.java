package org.firstinspires.ftc.teamcode.Threads;

import android.os.SystemClock;

import org.openftc.revextensions2.ExpansionHubMotor;


@Deprecated
public class LifterThreadPID implements Runnable {

    public static volatile boolean kill = false;
    private long lastMillis = 0;
    private volatile int currentTicks = 0;
    private volatile int lastTicks = 0;

    public static volatile boolean finished = true;

    public ExpansionHubMotor rightLifter;
    public ExpansionHubMotor leftLifter;

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
                finished = false;
                leftLifter.setTargetPosition(currentTicks);
                leftLifter.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);

                leftLifter.setPower(1);
                rightLifter.setPower(1);
                if (leftLifter.getCurrentPosition() < currentTicks) {
                    while (leftLifter.getCurrentPosition() < leftLifter.getTargetPosition()) {

                    }
                } else {
                    while (leftLifter.getCurrentPosition() > leftLifter.getTargetPosition()) {

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
        this.currentTicks = ticks;
    }
}
