package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.openftc.revextensions2.ExpansionHubMotor;

public class Robot extends OpMode {

    public boolean usingDebugger = false; //for debugging using UDP Unicast


    public ControllerInput controllerInputA;
    public ControllerInput controllerInputB;

    public Hardware robot = null;

    public ComputerDebugging computerDebugging;

    @Override
    public void init() {

        controllerInputA = new ControllerInput(gamepad1);
        controllerInputB = new ControllerInput(gamepad2);

        computerDebugging = new ComputerDebugging();

        robot = new Hardware();
        robot.init(hardwareMap);

    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

        robot.frontLeftWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRightWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeftWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.FLOAT);
        robot.backRightWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.FLOAT);

        robot.lifter.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop()
    {
        //computerDebugging.markEndOfUpdate();
        controllerInputB.update();
        controllerInputA.update();
    }

    @Override
    public void stop() {

    }
}