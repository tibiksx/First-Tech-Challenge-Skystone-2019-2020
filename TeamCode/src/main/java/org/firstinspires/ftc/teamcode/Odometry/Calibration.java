package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import java.io.File;

@TeleOp(name="Odometry Calibration", group = "Odometry")
public class Calibration extends LinearOpMode {

    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    final double pivotSpeed = 0.5;
    final double TICSK_PER_INCH = 100;
    double backTicksOffset = 0;

    File wheelsData = AppUtil.getInstance().getSettingsFile("wheelsData.txt");
    File backTicksFile = AppUtil.getInstance().getSettingsFile("backTicksFile.txt");

    ElapsedTime timer = new ElapsedTime();

    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){

        robot.init(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        initIMUParameters();

        imu.initialize(parameters);
        telemetry.addData("Odometry & IMU init","Complete");
        telemetry.update();

        timer.reset();
        waitForStart();
        sleep(100);

        while(getZAngle() < 90 && opModeIsActive())
        {


            if(getZAngle() < 60)
            {
                robot.frontRightWheel.setPower(-pivotSpeed);
                robot.backRightWheel.setPower(-pivotSpeed);
                robot.frontLeftWheel.setPower(pivotSpeed);
                robot.backLeftWheel.setPower(pivotSpeed);
            }
            else
            {
                robot.frontRightWheel.setPower(-pivotSpeed/2);
                robot.backRightWheel.setPower(-pivotSpeed/2);
                robot.frontLeftWheel.setPower(pivotSpeed/2);
                robot.backLeftWheel.setPower(pivotSpeed/2);
            }
            telemetry.addData("IMU Angle:",getZAngle());
            telemetry.update();
        }

        robot.frontRightWheel.setPower(0);
        robot.backRightWheel.setPower(0);
        robot.frontLeftWheel.setPower(0);
        robot.backLeftWheel.setPower(0);
        timer.reset();

        while (timer.milliseconds() < 1000 && opModeIsActive())
        {
            telemetry.addData("IMU Angle:",getZAngle());
            telemetry.update();
        }

        double angle = getZAngle();

        /* TODO Check encoder reversal */
        double encoderDifference = Math.abs(robot.leftEncoderMotor.getCurrentPosition()) +
                (Math.abs(robot.rightEncoderMotor.getCurrentPosition()));
        double backOffsetPerDeg = encoderDifference/angle;
        double wheelBaseSeparation = (2 * 90 * backOffsetPerDeg)/(Math.PI*TICSK_PER_INCH);
        backTicksOffset = robot.backEncoderMotor.getCurrentPosition()/Math.toRadians(getZAngle());

        ReadWriteFile.writeFile(wheelsData,String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(backTicksFile,String.valueOf(backTicksOffset));

        while(opModeIsActive())
        {
            telemetry.addData("Odometry Constants"," Calculated");
            telemetry.addData("Wheel Base Separation: ",wheelBaseSeparation);
            telemetry.addData("Back Encoder Offset: ",backTicksOffset);
            telemetry.addData("Constants saved","successfully");
            telemetry.update();
        }

    }

    private void initIMUParameters()
    {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "SensorBNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }

    private double getZAngle()
    {return (-imu.getAngularOrientation().firstAngle);}
}
