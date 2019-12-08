package org.firstinspires.ftc.teamcode.Threads;

import android.os.SystemClock;

import org.openftc.revextensions2.ExpansionHubMotor;


public class LifterThread implements Runnable {
    public static volatile boolean kill = false;
    private long lastMillis = 0;
    private volatile int currentTicks = 0;
    private volatile int lastTicks = 0;

    public static volatile boolean finished = true;

    public ExpansionHubMotor lifter;

    public LifterThread(ExpansionHubMotor lifter)
    {
        this.lifter = lifter;
    }

    @Override
    public void run() {
        while(true) {
            if (kill) {
                break;
            }

            //never run too fast
            if (SystemClock.uptimeMillis() - lastMillis < 50) {
                continue;
            }
            //set the last send time
            lastMillis = SystemClock.uptimeMillis();

            if(currentTicks != lastTicks) //we can move the motor to the new value
            {
                finished = false;
                double startTime = SystemClock.uptimeMillis();

                if(lifter.getCurrentPosition() < currentTicks)
                {
                    lifter.setPower(1);
                    while(lifter.getCurrentPosition() < currentTicks) {
                        if(SystemClock.uptimeMillis() - startTime > 1300) break;
                    }
                }
                else
                {
                    lifter.setPower(-1);
                    while(lifter.getCurrentPosition() > currentTicks) {
                        if(SystemClock.uptimeMillis() - startTime > 1300) break;
                    }
                }

                finished = true;
                lifter.setPower(0);
            }

            lastTicks = currentTicks;
        }
    }

    public void setTicks(int ticks){
        this.currentTicks = ticks;
    }
}
