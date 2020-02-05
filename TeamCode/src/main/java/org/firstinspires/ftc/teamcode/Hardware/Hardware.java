
package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

public class Hardware {

    //---------------WHEEL MOTORS-----------------------
    public ExpansionHubMotor frontLeftWheel = null;
    public ExpansionHubMotor frontRightWheel = null;
    public ExpansionHubMotor backLeftWheel = null;
    public ExpansionHubMotor backRightWheel = null;

    //----------------ENCODERS-----------------------------
    public ExpansionHubMotor verticalLeft = null;
    public ExpansionHubMotor verticalRight = null;
    public ExpansionHubMotor horizontal = null;

    //-------------------LIFTER-------------------------
    public ExpansionHubMotor leftLifter = null;
    public ExpansionHubMotor rightLifter = null;

    //--------------------SLIDER------------------
    public ExpansionHubMotor slider = null;

    //---------------------SENSORS-----------------
    public TouchSensor button = null;
    public BNO055IMU imu;

    //----------------------SERVOS----------------
    public ExpansionHubServo flipper1 = null;
    public ExpansionHubServo flipper2 = null;

    private HardwareMap hwMap = null;

    public Hardware() {
    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        //---------------------------WHEELS MOTORS----------------------------
        frontLeftWheel = hwMap.get(ExpansionHubMotor.class, "FL");
        frontRightWheel = hwMap.get(ExpansionHubMotor.class, "FR");
        backLeftWheel = hwMap.get(ExpansionHubMotor.class, "BL");
        backRightWheel = hwMap.get(ExpansionHubMotor.class, "BR");

        frontLeftWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftWheel.setDirection(ExpansionHubMotor.Direction.FORWARD);
        frontRightWheel.setDirection(ExpansionHubMotor.Direction.REVERSE);
        backLeftWheel.setDirection(ExpansionHubMotor.Direction.FORWARD);
        backRightWheel.setDirection(ExpansionHubMotor.Direction.REVERSE);

        frontLeftWheel.setPower(0.0);
        frontRightWheel.setPower(0.0);
        backLeftWheel.setPower(0.0);
        backRightWheel.setPower(0.0);


        //---------------------------ENCODERS-----------------------------------
        verticalLeft = hwMap.get(ExpansionHubMotor.class, "FR");
        verticalRight = hwMap.get(ExpansionHubMotor.class, "lifterR");
        horizontal = hwMap.get(ExpansionHubMotor.class, "BR");

        verticalLeft.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        verticalRight.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        horizontal.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);

        verticalLeft.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);


        //-------------------------LIFTER-------------------------------------
        leftLifter = hwMap.get(ExpansionHubMotor.class, "lifterL");
        rightLifter = hwMap.get(ExpansionHubMotor.class, "lifterR");

        leftLifter.setPower(0.0);
        rightLifter.setPower(0.0);

        leftLifter.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLifter.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLifter.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        rightLifter.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);

//        leftLifter.setDirection(ExpansionHubMotor.Direction.REVERSE);
//        rightLifter.setDirection(ExpansionHubMotor.Direction.REVERSE);

        leftLifter.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        rightLifter.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);

        //-------------------------------SLIDER-----------------------------
        slider = hwMap.get(ExpansionHubMotor.class, "slide");
        slider.setPower(0.0);
        slider.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        slider.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        slider.setDirection(ExpansionHubMotor.Direction.FORWARD);


        //-------------------SENSORS-------------------------------(imu is separate from this)
        button = hwMap.get(TouchSensor.class, "touch");

        //-----------------------SERVOS----------------------------
//        flipper1 = hwMap.get(ExpansionHubServo.class, " flipper1");
//        flipper2 = hwMap.get(ExpansionHubServo.class, "flipper2");

    }

    public void initIMU() {
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

}


