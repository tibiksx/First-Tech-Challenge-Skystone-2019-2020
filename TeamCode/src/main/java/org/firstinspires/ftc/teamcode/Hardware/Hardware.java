
package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ComputerVision.SkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

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
    public ExpansionHubServo posLeft = null;
    public ExpansionHubServo posRight = null;
    public ExpansionHubServo claw = null;
    public ExpansionHubServo flipper = null;
    public ExpansionHubServo foundationLeft = null;
    public ExpansionHubServo foundationRight =null;

    //---------CAMERA-----------------------------
    public OpenCvCamera webcam;
    public int cameraMonitorViewId;

    //----------Expansion Hubs-----------------
    public ExpansionHubEx ExpansionHub1;
    public ExpansionHubEx ExpansionHub2;

    //------------Bulk Data---------------------
    public RevBulkData ExpansionHub1BulkData;
    public RevBulkData ExpansionHub2BulkData;

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

        frontLeftWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);


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
        posRight = hwMap.get(ExpansionHubServo.class, "posRight");
        posLeft = hwMap.get(ExpansionHubServo.class, "posLeft");
        claw = hwMap.get(ExpansionHubServo.class,"claw");
        flipper = hwMap.get(ExpansionHubServo.class,"flipper");
        foundationLeft = hwMap.get(ExpansionHubServo.class,"foundLeft");
        foundationRight = hwMap.get(ExpansionHubServo.class,"foundRight");

        //----------------------Expansion Hubs------------------------
        ExpansionHub1 = hwMap.get(ExpansionHubEx.class,"Expansion Hub 1");
        ExpansionHub2 = hwMap.get(ExpansionHubEx.class,"Expansion Hub 2");
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

    public void initWebcam() {
        cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }


}


