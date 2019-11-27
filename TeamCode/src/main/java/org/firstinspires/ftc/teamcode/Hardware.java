
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;

    public DcMotor test1 = null;
    public DcMotor test2 = null;

    public DcMotor slider = null;
    public DcMotor lifter = null;

    public Servo foundation1 = null;
    public Servo foundation2 = null;

    public Servo fliper2 = null;
    public Servo fliper1 = null;

    HardwareMap hwMap           =  null;

    public Hardware(){

    }


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        frontLeftWheel = hwMap.get(DcMotor.class, "FL");
        frontRightWheel = hwMap.get(DcMotor.class, "FR");
        backLeftWheel = hwMap.get(DcMotor.class, "BL");
        backRightWheel = hwMap.get(DcMotor.class, "BR");

        test1 = hwMap.get(DcMotor.class, "motor1");
        test2 = hwMap.get(DcMotor.class, "motor2");

        slider = hwMap.get(DcMotor.class, "slide");
        lifter = hwMap.get(DcMotor.class, "lift");

        foundation1 = hwMap.get(Servo.class, "fund1");
        foundation2 = hwMap.get(Servo.class, "fund2");
        fliper1 = hwMap.get(Servo.class, "flip1");
        fliper2 = hwMap.get(Servo.class, "flip2");

        test1.setPower(0.0);
        test2.setPower(0.0);

        frontLeftWheel.setPower(0.0);
        frontRightWheel.setPower(0.0);
        backLeftWheel.setPower(0.0);
        backRightWheel.setPower(0.0);
        fliper2.setPosition(0.0);

        slider.setPower(0.0);
        lifter.setPower(0.0);

        foundation1.setPosition(1.0);
        foundation2.setPosition(0.0);
        fliper1.setPosition(0.90);

        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        test1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        test2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        test1.setDirection(DcMotor.Direction.FORWARD);
        test2.setDirection(DcMotor.Direction.FORWARD);

        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        slider.setDirection(DcMotor.Direction.FORWARD);

        lifter.setDirection(DcMotor.Direction.REVERSE);

        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

 }

