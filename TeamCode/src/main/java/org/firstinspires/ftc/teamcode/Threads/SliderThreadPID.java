package org.firstinspires.ftc.teamcode.Threads;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.Misc.LifterMethods;
import org.openftc.revextensions2.ExpansionHubMotor;

public class SliderThreadPID implements Runnable {

    public static volatile boolean kill = false;
    private long lastMillis = 0;
    private volatile int currentTicks = 0;
    private volatile int lastTicks = 0;

    public static volatile boolean finished = true;

    private ExpansionHubMotor slider;

    public SliderThreadPID(ExpansionHubMotor slider) {
        this.slider = slider;
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
                if(currentTicks < lastTicks) { //backward using PID
                    slider.setTargetPosition(currentTicks);
                    slider.setTargetPositionTolerance(70);
                    slider.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
                }
                finished = false;
                if (slider.getCurrentPosition() < currentTicks) {
                    slider.setPower(1);
                    while (slider.getCurrentPosition() < currentTicks - 60 && !finished) {

                    }
                } else {
                    slider.setPower(-1);
                    while (slider.getCurrentPosition() > currentTicks + 60 && !finished) {
                    }
                }
                slider.setPower(0);
                finished = true;
                slider.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
            }

            lastTicks = currentTicks;
        }
    }

    public void setTicks(int ticks) {
        currentTicks = ticks;
    }
}
