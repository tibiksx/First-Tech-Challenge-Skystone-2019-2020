package org.firstinspires.ftc.teamcode.Threads;

import android.os.SystemClock;

import org.openftc.revextensions2.ExpansionHubMotor;


public class LifterThread implements Runnable {
    public static volatile boolean kill = false;
    private long lastMillis = 0;
    private volatile int currentTicks = 0;
    private volatile int lastTicks = 0;
    private volatile double power = 1;

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
                lifter.setTargetPosition(currentTicks);
                lifter.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
                lifter.setPower(power);
                finished = false;
                double startTime = SystemClock.uptimeMillis();

                if(lifter.getCurrentPosition() < currentTicks)
                {
                    while(lifter.getCurrentPosition() < lifter.getTargetPosition()) {
                        if(SystemClock.uptimeMillis() - startTime > 1000) break;
                    }
                }
                else
                {
                    while(lifter.getCurrentPosition() > lifter.getTargetPosition()) {
                        if(SystemClock.uptimeMillis() - startTime > 1000) break;
                    }
                }

                finished = true;
                lifter.setPower(0);
                lifter.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
            }

            lastTicks = currentTicks;
        }
    }

    public void setTicks(int ticks){
        this.currentTicks = ticks;
    }
}
