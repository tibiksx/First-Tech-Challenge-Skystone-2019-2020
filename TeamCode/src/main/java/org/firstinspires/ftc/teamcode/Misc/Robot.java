package org.firstinspires.ftc.teamcode.Misc;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Robot extends OpMode {

    public boolean usingDebugger = false; //for debugging using UDP Unicast

    public long currTimeMillis = 0; //time in ms

    public ControllerInput controllerInputA;
    public ControllerInput controllerInputB;

    public Hardware robot = null;
    public HardwareMap hw = null;
    @Override
    public void init() {
        currTimeMillis = SystemClock.uptimeMillis();

        controllerInputA = new ControllerInput(gamepad1);
        controllerInputB = new ControllerInput(gamepad2);

        robot = new Hardware();
        robot.init(hw);
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop()
    {

    }
}