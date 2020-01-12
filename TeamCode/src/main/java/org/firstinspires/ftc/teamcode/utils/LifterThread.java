package org.firstinspires.ftc.teamcode.utils;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LifterThread implements Runnable {

    public static volatile boolean kill = false;
    private long lastMillis = 0;
    private volatile int currentTicks = 0;
    private volatile int lastTicks = 0;

    public static volatile boolean finished = true;

    public DcMotor lifter1;
    public DcMotor lifter2;

    public LifterThread(DcMotor lifter1, DcMotor lifter2) {
        this.lifter1 = lifter1;
        this.lifter2 = lifter2;
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

                if (currentTicks < lifter1.getCurrentPosition()) {
                    lifter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lifter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    finished = false;

                    lifter1.setPower(-1);
                    lifter2.setPower(-1);

                    while(lifter1.getCurrentPosition() > Math.abs(currentTicks)) {
                        if (lifter1.getCurrentPosition() - Math.abs(currentTicks) < 450) {
                            lifter1.setPower(-(lifter1.getCurrentPosition() - (double)Math.abs(currentTicks) / 100));
                            lifter2.setPower(-(lifter1.getCurrentPosition() - (double)Math.abs(currentTicks) / 100));
                        }

                        if (lifter1.getCurrentPosition() - Math.abs(currentTicks) < 500)
                            break;
                    }

                    finished = true;

                    lifter1.setPower(0);
                    lifter2.setPower(0);

                } else {

                    lifter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lifter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    finished = false;

                    lifter1.setPower(1);
                    lifter2.setPower(1);

                    while(lifter1.getCurrentPosition() < Math.abs(currentTicks)) {

                        if (Math.abs(currentTicks) - lifter1.getCurrentPosition() < 100) {
                            lifter1.setPower((Math.abs(currentTicks) - lifter1.getCurrentPosition()) / 100);
                            lifter2.setPower((Math.abs(currentTicks) - lifter1.getCurrentPosition()) / 100);
                        }

                        if (Math.abs(currentTicks) - lifter1.getCurrentPosition() < 30)
                            break;
                    }

                    finished = true;

                    lifter1.setPower(0);
                    lifter2.setPower(0);

                }

            }

            lastTicks = currentTicks;

            lifter1.setDirection(DcMotor.Direction.REVERSE);
            lifter2.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void setTicks(int ticks){
        this.currentTicks = ticks;
    }
}