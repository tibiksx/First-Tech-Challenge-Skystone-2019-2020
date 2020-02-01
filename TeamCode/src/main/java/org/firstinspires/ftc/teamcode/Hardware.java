
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;

    public DcMotor verticalLeft, verticalRight, horizontal;

    public DcMotor lifterLeft = null;
    public DcMotor lifterRight = null;

    public DcMotor slider = null;

    public DigitalChannel magnet = null;
    public BNO055IMU imu;

    HardwareMap hwMap           =  null;

    public Hardware() {}

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        frontLeftWheel = hwMap.get(DcMotor.class, "FL");
        frontRightWheel = hwMap.get(DcMotor.class, "FR");
        backLeftWheel = hwMap.get(DcMotor.class, "BL");
        backRightWheel = hwMap.get(DcMotor.class, "BR");

        verticalLeft = hwMap.get(DcMotor.class, "FR");
        verticalRight = hwMap.get(DcMotor.class, "lifterR");
        horizontal = hwMap.get(DcMotor.class, "BR");

        lifterLeft = hwMap.get(DcMotor.class, "lifterL");
        lifterRight = hwMap.get(DcMotor.class, "lifterR");

        slider = hwMap.get(DcMotor.class, "slide");
        
        imu = hwMap.get(BNO055IMU.class, "imu");
        magnet = hwMap.get(DigitalChannel.class, "magnet");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        frontLeftWheel.setPower(0.0);
        frontRightWheel.setPower(0.0);
        backLeftWheel.setPower(0.0);
        backRightWheel.setPower(0.0);

        lifterLeft.setPower(0.0);
        lifterRight.setPower(0.0);

        slider.setPower(0.0);

        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lifterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lifterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        lifterLeft.setDirection(DcMotor.Direction.REVERSE);
        lifterRight.setDirection(DcMotor.Direction.REVERSE);

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        lifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setDirection(DcMotorSimple.Direction.FORWARD);
    }

 }



