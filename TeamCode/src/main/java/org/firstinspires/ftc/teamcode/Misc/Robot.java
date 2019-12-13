package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class Robot extends OpMode {

    public boolean usingDebugger = false; //for debugging using UDP Unicast


    public ControllerInput controllerInputA;
    public ControllerInput controllerInputB;

    public Hardware robot = null;

    public ComputerDebugging computerDebugging;

    public ExpansionHubEx revMaster = null;
    public ExpansionHubEx revSlave = null;


    protected final int TICKS_PER_REV = 1600;

    public Encoder rightEncoder = null;
    public Encoder leftEncoder = null;
    public Encoder backEncoder = null;

    @Override
    public void init() {

        controllerInputA = new ControllerInput(gamepad1);
        controllerInputB = new ControllerInput(gamepad2);

        computerDebugging = new ComputerDebugging();

        robot = new Hardware();
        robot.init(hardwareMap);

        revMaster = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 1");
        revSlave = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");

        leftEncoder = new Encoder(robot.leftEncoderMotor,TICKS_PER_REV);
//        backEncoder = new Encoder(robot.backEncoderMotor,ticksPerRev);
//        rightEncoder = new Encoder(robot.rightEncoderMotor, ticksPerRev);

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