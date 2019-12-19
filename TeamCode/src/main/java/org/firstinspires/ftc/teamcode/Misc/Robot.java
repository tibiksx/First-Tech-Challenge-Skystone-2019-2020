package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class Robot extends OpMode {



    protected ControllerInput controllerInputA;
    protected ControllerInput controllerInputB;

    protected Hardware robot = null;

    protected Encoder rightEncoder = null;
    protected Encoder leftEncoder = null;
    protected Encoder backEncoder = null;

    protected ExpansionHubEx revSlave = null;
    protected ExpansionHubEx revMaster = null;
    protected RevBulkData slaveBulkData = null;
    protected RevBulkData masterBulkData = null;

    @Override
    public void init() {

        controllerInputA = new ControllerInput(gamepad1);
        controllerInputB = new ControllerInput(gamepad2);

        revMaster = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 1");
        revSlave = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");

        slaveBulkData = revSlave.getBulkInputData();
        masterBulkData = revMaster.getBulkInputData();

        robot = new Hardware();
        robot.init(hardwareMap);

        leftEncoder = new Encoder(robot.leftEncoderMotor);
        //backEncoder = new Encoder(robot.backEncoderMotor);
        //rightEncoder = new Encoder(robot.rightEncoderMotor);


        robot.frontLeftWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRightWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeftWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.FLOAT);
        robot.backRightWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.FLOAT);

        robot.lifter.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop()
    {
        controllerInputB.update();
        controllerInputA.update();

        slaveBulkData = revSlave.getBulkInputData();
        masterBulkData = revMaster.getBulkInputData();

        leftEncoder.updateEncoder(masterBulkData.getMotorCurrentPosition(robot.leftEncoderMotor));

    }

    @Override
    public void stop() {

    }
}