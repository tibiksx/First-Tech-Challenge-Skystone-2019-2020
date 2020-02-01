package org.firstinspires.ftc.teamcode.utils;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class LifterThread implements Runnable {

    private static volatile boolean kill = false;
    private long lastMillis = 0;
    private volatile int currentTicks = 0;
    private volatile int lastTicks = 0;

    public static volatile boolean finished = true;

    private DcMotor lifter1;
    private DcMotor lifter2;

    public LifterThread(DcMotor lifter1, DcMotor lifter2) {
        this.lifter1 = lifter1;
        this.lifter2 = lifter2;
    }

    @Override
    public void run() {

        lifter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!kill) {

            if (SystemClock.uptimeMillis() - lastMillis < 50) {
                continue;
            }

            lastMillis = SystemClock.uptimeMillis();

            if (currentTicks != lastTicks) {

                finished = false;

                if (currentTicks < lifter1.getCurrentPosition()) {

                    lifter1.setPower(-1);
                    lifter2.setPower(-1);

                    while(lifter1.getCurrentPosition() > Math.abs(currentTicks)) {

                        lifter1.setPower(-Math.cos(Math.toRadians(Range.scale(lifter1.getCurrentPosition(), lastTicks, currentTicks, 0, 90))));

                        if (!(lifter1.getCurrentPosition() > 300)) {
                            lifter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            lifter1.setPower(0);
                            lifter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            lifter2.setPower(0);
                            break;
                        }

                    }

                    finished = true;

                    lifter1.setPower(0);
                    lifter2.setPower(0);

                } else {

                    finished = false;

                    lifter1.setPower(1);
                    lifter2.setPower(1);

                    while(lifter1.getCurrentPosition() < Math.abs(currentTicks)) {

                        lifter1.setPower(Math.cos(Math.toRadians(Range.scale(lifter1.getCurrentPosition(), lastTicks, currentTicks, 0, 90))));
                    }

                    finished = true;

                    lifter1.setPower(0);
                    lifter2.setPower(0);

                }

            }

            lastTicks = lifter1.getCurrentPosition();

            lifter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lifter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            lifter1.setDirection(DcMotor.Direction.REVERSE);
            lifter2.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void setTicks(int ticks){
        this.currentTicks = ticks;
    }

}