package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Sample created by Sarthak on 6/1/2019.
 * Modified and implemented by Tiberiu Ureche on 12/01/2020.
 */
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends LinearOpMode {

    Hardware robot = new Hardware();

    final double PIVOT_SPEED = 0.4;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    final double COUNTS_PER_INCH = 1600 / (Math.PI * 1.96);

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(getZAngle() < 90 && opModeIsActive()){

            robot.frontRightWheel.setPower(-PIVOT_SPEED);
            robot.backRightWheel.setPower(-PIVOT_SPEED);
            robot.frontLeftWheel.setPower(PIVOT_SPEED);
            robot.backLeftWheel.setPower(PIVOT_SPEED);

            if(getZAngle() < 60) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            } else {
                setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(robot.verticalLeft.getCurrentPosition() - robot.verticalRight.getCurrentPosition());

        double verticalEncoderTickOffsetPerDegree = encoderDifference / angle;

        double wheelBaseSeparation = (180 * verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = robot.horizontal.getCurrentPosition()/getZAngle();

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    private double getZAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle);
    }

    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        robot.frontRightWheel.setPower(rf);
        robot.backRightWheel.setPower(rb);
        robot.frontLeftWheel.setPower(lf);
        robot.backLeftWheel.setPower(lb);
    }

}


