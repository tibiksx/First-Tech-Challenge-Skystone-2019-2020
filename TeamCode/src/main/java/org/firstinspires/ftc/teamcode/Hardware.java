
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;

    //public DcMotor lifter = null;

    HardwareMap hwMap           =  null;

    public Hardware(){

    }


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        frontLeftWheel = hwMap.get(DcMotor.class, "FL");
        frontRightWheel = hwMap.get(DcMotor.class, "FR");
        backLeftWheel = hwMap.get(DcMotor.class, "BL");
        backRightWheel = hwMap.get(DcMotor.class, "BR");

        //lifter = hwMap.get(DcMotor.class, "lift");

        frontLeftWheel.setPower(0.0);
        frontRightWheel.setPower(0.0);
        backLeftWheel.setPower(0.0);
        backRightWheel.setPower(0.0);

        //lifter.setPower(0);

        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        //lifter.setDirection(DcMotor.Direction.REVERSE);

        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

 }

