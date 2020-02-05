package org.firstinspires.ftc.teamcode.Threads;

import android.os.SystemClock;


import org.openftc.revextensions2.ExpansionHubMotor;
public class MotorThread implements Runnable {


    public ExpansionHubMotor motor;
    public int ticks = -1;
    public float time = -1;
    double power;
    long currentTime;
    public MotorThread(ExpansionHubMotor motor,int ticks, double power)
    {
        this.motor = motor;
        this.ticks = ticks;
        this.power = power;
    }

    public MotorThread(ExpansionHubMotor motor, float time, double power)
    {
        this.motor = motor;
        this.time = time;
        this.power = power;
    }

    @Override
    public void run() {
        if(ticks != -1) //move with ticks
        {
            motor.setTargetPosition(ticks);
            motor.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
            double oldTime = SystemClock.uptimeMillis();
            while (motor.getCurrentPosition() < motor.getTargetPosition()) {
                currentTime = SystemClock.uptimeMillis();
                if(currentTime - oldTime > 3000) break;
            }
            motor.setPower(0.0);
            motor.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        }
        if(time != -1) //move with time
        {
            currentTime = SystemClock.uptimeMillis();
            motor.setPower(power);
            while(SystemClock.uptimeMillis() < currentTime + time) { }
            motor.setPower(0);
        }
    }
}
