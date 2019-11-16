package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

public class Hardware {

    public ExpansionHubMotor br = null;
    public ExpansionHubMotor bl = null;
    public ExpansionHubMotor fr = null;
    public ExpansionHubMotor fl = null;

    public ExpansionHubMotor Lifter = null;

    public ExpansionHubEx revMaster;
    public ExpansionHubEx revSlave;
    HardwareMap hw = null;


    public ExpansionHubMotor encoderLeft = null;
    public ExpansionHubMotor encoderRight = null;
    public ExpansionHubMotor encoderBack = null;

    public Hardware()
    {

    }

    public void init(HardwareMap hw)
    {
        this.hw = hw;

        //---------------MOTORS & ENCODERS--------------------
        revMaster = hw.get(ExpansionHubEx.class, "Expansion Hub 1");
        revSlave = hw.get(ExpansionHubEx.class,"Expansion Hub 2");
        fl =(ExpansionHubMotor)hw.get("FL");
        fr = (ExpansionHubMotor)hw.get("FR");
        bl = (ExpansionHubMotor)hw.get("BL");
        br = (ExpansionHubMotor)hw.get("BR");

        Lifter = (ExpansionHubMotor)hw.get("lifter");
        //encoderLeft = (ExpansionHubMotor)hw.get("encoderLeft");
        //encoderRight = (ExpansionHubMotor)hw.get("encoderRight");
        //encoderBack = (ExpansionHubMotor)hw.get("encoderBack");
        //----------------------------------------------

        fl.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);

        Lifter.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lifter.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);

        //encoderLeft.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        //encoderBack.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        //encoderRight.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        //encoderLeft.setDirection(ExpansionHubMotor.Direction.REVERSE);

        fl.setPower(0.0);
        fr.setPower(0.0);
        br.setPower(0.0);
        bl.setPower(0.0);
        Lifter.setPower(0.0);
    }
}
