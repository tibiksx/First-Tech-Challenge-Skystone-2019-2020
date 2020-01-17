
package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;

public class Hardware {

    //-------------------------WHEEL MOTORS-------------------
    public ExpansionHubMotor frontLeftWheel = null;
    public ExpansionHubMotor frontRightWheel = null;
    public ExpansionHubMotor backLeftWheel = null;
    public ExpansionHubMotor backRightWheel = null;

    //-----------------------LIFTER MOTORS-----------------
    public ExpansionHubMotor leftLifter = null;
    public ExpansionHubMotor rightLifter = null;
    //public ExpansionHubMotor extension = null;

    //-----------------------ENCODER MOTORS--------------------------
    public ExpansionHubMotor leftEncoderMotor = null;
    public ExpansionHubMotor backEncoderMotor = null;
    public ExpansionHubMotor rightEncoderMotor = null;


    private HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        frontLeftWheel = hwMap.get(ExpansionHubMotor.class, "FL");
        frontRightWheel = hwMap.get(ExpansionHubMotor.class, "FR");
        backLeftWheel = hwMap.get(ExpansionHubMotor.class, "BL");
        backRightWheel = hwMap.get(ExpansionHubMotor.class, "BR");

        leftLifter = hwMap.get(ExpansionHubMotor.class, "lifterL");
        rightLifter = hwMap.get(ExpansionHubMotor.class, "lifterR");
        //extension = hwMap.get(ExpansionHubMotor.class,"extension");

        leftEncoderMotor = hwMap.get(ExpansionHubMotor.class, "FR");
        rightEncoderMotor = hwMap.get(ExpansionHubMotor.class, "lifterR");
        backEncoderMotor = hwMap.get(ExpansionHubMotor.class, "BR");


        leftEncoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        rightEncoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        backEncoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);

        leftEncoderMotor.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoderMotor.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEncoderMotor.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        backEncoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftWheel.setPower(0.0);
        frontRightWheel.setPower(0.0);
        backLeftWheel.setPower(0.0);
        backRightWheel.setPower(0.0);

        rightLifter.setPower(0.0);
        leftLifter.setPower(0.0);
        //extension.setPower(0.0);

        frontLeftWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLifter.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER); //only the left encoder is enough
        leftLifter.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        //extension.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);

        frontLeftWheel.setDirection(ExpansionHubMotor.Direction.FORWARD);
        frontRightWheel.setDirection(ExpansionHubMotor.Direction.REVERSE);
        backLeftWheel.setDirection(ExpansionHubMotor.Direction.FORWARD);
        backRightWheel.setDirection(ExpansionHubMotor.Direction.REVERSE);

//        rightLifter.setDirection(ExpansionHubMotor.Direction.REVERSE);
//        leftLifter.setDirection(ExpansionHubMotor.Direction.REVERSE);
    }

}
