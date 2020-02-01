package org.firstinspires.ftc.teamcode.utils;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class SliderThread implements Runnable {

    DcMotor slide;

    private volatile int mode;

    private static volatile boolean kill = false;
    private long lastMillis = 0;
    private volatile int currentTicks = 0;
    private volatile int lastTicks = 0;
    public static volatile boolean finished = true;

    public SliderThread(DcMotor slide, int mode) {
        this.slide = slide;
        this.mode = mode;
    }

    @Override
    public void run() {

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!kill) {

            if (SystemClock.uptimeMillis() - lastMillis < 50) {
                continue;
            }

            lastMillis = SystemClock.uptimeMillis();

            if (currentTicks != lastTicks) {

                finished = false;

                if (mode == 1) {

                    while(slide.getCurrentPosition() < Math.abs(currentTicks)) {

                        slide.setPower(-Math.cos(Math.toRadians(Range.scale(slide.getCurrentPosition(), lastTicks, currentTicks, 0, 90))));

                    }

                    finished = true;

                    slide.setPower(0);

                } else if (mode == 0) {

                    while(slide.getCurrentPosition() > Math.abs(lastTicks)) {

                        slide.setPower(Math.cos(Math.toRadians(Range.scale(slide.getCurrentPosition(), lastTicks, currentTicks, 0, 90))));

                    }

                    finished = true;

                    slide.setPower(0);
                }
            }

            lastTicks = slide.getCurrentPosition();
        }

    }

    public void setTicks(int ticks){
        this.currentTicks = ticks;
    }
}
