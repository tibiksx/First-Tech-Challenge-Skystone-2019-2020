package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import org.openftc.revextensions2.ExpansionHubMotor;

/**
 *  This is the baseline for all IterativeOpModes. This is the only class that extends from
 *  OpMode and it allows for quick creation of new OpModes and hides away some initializing,
 *  making code much easier to work with and understand.
 */

public class Robot extends OpMode {


    protected ControllerInput controllerInputA;
    protected ControllerInput controllerInputB;

    protected Hardware robot = null;

    protected Encoder rightEncoder = null;
    protected Encoder leftEncoder = null;
    protected Encoder backEncoder = null;

    @Override
    public void init() {

        controllerInputA = new ControllerInput(gamepad1);
        controllerInputB = new ControllerInput(gamepad2);

        robot = new Hardware();
        robot.init(hardwareMap);

        leftEncoder = new Encoder(robot.leftEncoderMotor);
        backEncoder = new Encoder(robot.backEncoderMotor);
        rightEncoder = new Encoder(robot.rightEncoderMotor);

        leftEncoder.updateEncoder();
        rightEncoder.updateEncoder();
        backEncoder.updateEncoder();


        robot.frontLeftWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        controllerInputB.update();
        controllerInputA.update();

        leftEncoder.updateEncoder();
        rightEncoder.updateEncoder();
        backEncoder.updateEncoder();
    }

    @Override
    public void stop() {

    }
}