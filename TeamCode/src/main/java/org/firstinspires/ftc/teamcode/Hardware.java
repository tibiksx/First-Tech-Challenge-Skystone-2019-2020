
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;

    public DcMotor verticalLeft, verticalRight, horizontal;

    public DcMotor slider = null;
    public DcMotor lifter1 = null;
    public DcMotor lifter2 = null;

    public Servo foundation1 = null;
    public Servo foundation2 = null;

    public Servo fliper2 = null;
    public Servo fliper1 = null;

    public BNO055IMU imu;

    HardwareMap hwMap           =  null;

    public Hardware(){

    }


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        frontLeftWheel = hwMap.get(DcMotor.class, "FL");
        frontRightWheel = hwMap.get(DcMotor.class, "FR");
        backLeftWheel = hwMap.get(DcMotor.class, "BL");
        backRightWheel = hwMap.get(DcMotor.class, "BR");

        verticalLeft = hwMap.get(DcMotor.class, "FL");
        verticalRight = hwMap.get(DcMotor.class, "FR");
        horizontal = hwMap.get(DcMotor.class, "BL");

        slider = hwMap.get(DcMotor.class, "slide");
        lifter1 = hwMap.get(DcMotor.class, "lift1");
        lifter2 = hwMap.get(DcMotor.class, "lift2");

        foundation1 = hwMap.get(Servo.class, "fund1");
        foundation2 = hwMap.get(Servo.class, "fund2");
        fliper1 = hwMap.get(Servo.class, "flip1");
        fliper2 = hwMap.get(Servo.class, "flip2");

        imu = hwMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        frontLeftWheel.setPower(0.0);
        frontRightWheel.setPower(0.0);
        backLeftWheel.setPower(0.0);
        backRightWheel.setPower(0.0);
        fliper2.setPosition(0.0);

        slider.setPower(0.0);
        lifter1.setPower(0.0);
        lifter2.setPower(0.0);

        foundation1.setPosition(1.0);
        foundation2.setPosition(0.0);
        fliper1.setPosition(0.90);

        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // left encoder
        frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // right encoder
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // back encoder
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lifter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lifter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        slider.setDirection(DcMotor.Direction.FORWARD);

        lifter1.setDirection(DcMotor.Direction.REVERSE);
        lifter2.setDirection(DcMotor.Direction.REVERSE);

        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

 }

