package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LifterThread implements Runnable {

    public static volatile boolean kill = false;
    private long lastMillis = 0;
    private volatile int currentTicks = 0;
    private volatile int lastTicks = 0;

    public static volatile boolean finished = true;

    public DcMotor lifter;

    public LifterThread(DcMotor lifter)
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

            if(currentTicks != lastTicks) {

                if (currentTicks < lifter.getCurrentPosition()) {
                    lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    finished = false;

                    lifter.setPower(-1);

                    while(lifter.getCurrentPosition() > Math.abs(currentTicks)) {
                        if (lifter.getCurrentPosition() - Math.abs(currentTicks) < 450) {
                            lifter.setPower(-(lifter.getCurrentPosition() - (double)Math.abs(currentTicks) / 100));
                        }

                        if (lifter.getCurrentPosition() - Math.abs(currentTicks) < 500)
                            break;
                    }

                    finished = true;
                    lifter.setPower(0);

                    lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    finished = false;

                    lifter.setPower(1);

                    while(lifter.getCurrentPosition() < Math.abs(currentTicks)) {
                        if (Math.abs(currentTicks) - lifter.getCurrentPosition() < 100) {
                            lifter.setPower((Math.abs(currentTicks) - lifter.getCurrentPosition()) / 100);
                        }

                        if (Math.abs(currentTicks) - lifter.getCurrentPosition() < 30)
                            break;
                    }

                    finished = true;
                    lifter.setPower(0);

                    lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

            }

            lastTicks = currentTicks;

            lifter.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void setTicks(int ticks){
        this.currentTicks = ticks;
    }
}