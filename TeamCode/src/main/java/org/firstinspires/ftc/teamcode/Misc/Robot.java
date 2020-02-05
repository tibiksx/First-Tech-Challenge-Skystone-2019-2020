package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

/**
 *  This is the baseline for all Iterative OpModes. This is the only class that extends from
 *  OpMode and it allows for quick creation of new OpModes and hides away some initializing,
 *  making code much easier to work with and understand.
 */

public class Robot extends OpMode {


    protected ControllerInput controllerInputA;
    protected ControllerInput controllerInputB;

    protected Hardware robot = null;


    @Override
    public void init() {

        controllerInputA = new ControllerInput(gamepad1);
        controllerInputB = new ControllerInput(gamepad2);

        robot = new Hardware();
        robot.init(hardwareMap);

    }

    @Override
    public void loop() {
        controllerInputB.update();
        controllerInputA.update();
    }

    @Override
    public void stop() {

    }
}