
package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;


public class Hardware {

    //-------------------------WHEEL MOTORS-------------------
    public ExpansionHubMotor frontLeftWheel = null;
    public ExpansionHubMotor frontRightWheel = null;
    public ExpansionHubMotor backLeftWheel = null;
    public ExpansionHubMotor backRightWheel = null;

    //-----------------------ADDITIONAL MOTORS-----------------
    public ExpansionHubMotor slider = null;
    public ExpansionHubMotor lifter = null;

    //----------------------FOUNDATION SERVOS------------------
    public ExpansionHubServo foundation1 = null;
    public ExpansionHubServo foundation2 = null;

    //----------------------COLLECTER SERVOS-------------------
    public ExpansionHubServo flipper2 = null;
    public ExpansionHubServo flipper1 = null;

    //-----------------------ENCODERS--------------------------
    //public ExpansionHubMotor leftEncoderMotor = null;
    //public ExpansionHubMotor backEncoderMotor = null;
    public ExpansionHubMotor rightEncoderMotor = null;

    //-----------------------TEST MOTORS-----------------------
    public ExpansionHubMotor testMotor = null;


    HardwareMap hwMap = null;

    public Hardware()
    { }


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        frontLeftWheel = hwMap.get(ExpansionHubMotor.class, "FL");
        frontRightWheel = hwMap.get(ExpansionHubMotor.class, "FR");
        backLeftWheel = hwMap.get(ExpansionHubMotor.class, "BL");
        backRightWheel = hwMap.get(ExpansionHubMotor.class, "BR");

        slider = hwMap.get(ExpansionHubMotor.class, "slide");
        lifter = hwMap.get(ExpansionHubMotor.class, "lift");

        foundation1 = hwMap.get(ExpansionHubServo.class, "fund1");
        foundation2 = hwMap.get(ExpansionHubServo.class, "fund2");
        flipper1 = hwMap.get(ExpansionHubServo.class, "flip1");
        flipper2 = hwMap.get(ExpansionHubServo.class, "flip2");

        //backEncoder = hwMap.get(ExpansionHubMotor.class,"backEncoder");
        //leftEncoder = hwMap.get(ExpansionHubMotor.class,"leftEncoder");
        rightEncoderMotor = hwMap.get(ExpansionHubMotor.class,"motor1");

        testMotor = hwMap.get(ExpansionHubMotor.class,"motor2");

        frontLeftWheel.setPower(0.0);
        frontRightWheel.setPower(0.0);
        backLeftWheel.setPower(0.0);
        backRightWheel.setPower(0.0);
        flipper2.setPosition(0.0);

        slider.setPower(0.0);
        lifter.setPower(0.0);

        foundation1.setPosition(1.0);
        foundation2.setPosition(0.0);
        flipper1.setPosition(0.90);

        frontLeftWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);

        lifter.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);//might reset when lifter is half-way up, messes up the ticks
        lifter.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);

        frontLeftWheel.setDirection(ExpansionHubMotor.Direction.FORWARD);
        frontRightWheel.setDirection(ExpansionHubMotor.Direction.REVERSE);
        backLeftWheel.setDirection(ExpansionHubMotor.Direction.FORWARD);
        backRightWheel.setDirection(ExpansionHubMotor.Direction.REVERSE);
        slider.setDirection(ExpansionHubMotor.Direction.FORWARD);
        lifter.setDirection(ExpansionHubMotor.Direction.REVERSE);


    }

}
