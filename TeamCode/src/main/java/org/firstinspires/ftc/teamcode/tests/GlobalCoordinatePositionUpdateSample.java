package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.utils.ControllerInput;

/**
 * Sample created by Sarthak on 6/1/2019.
 * Modified and implemented by Tiberiu Ureche on 12/01/2020.
 */
@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
public class GlobalCoordinatePositionUpdateSample extends LinearOpMode {

    Hardware robot = new Hardware();

    double coeff = 1.0;

    public ControllerInput controllerInputA;
    public ControllerInput controllerInputB;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = 1600 / (Math.PI * 1.96);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        //Reset the encoders
        robot.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        robot.verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        controllerInputA = new ControllerInput(gamepad1);
        controllerInputB = new ControllerInput(gamepad2);

        while(opModeIsActive()){

            sleep(2000);
            do {
                setPowerAll(-Math.cos(Math.toRadians(globalPositionUpdate.returnOrientation())) * 1.1, -Math.cos(Math.toRadians(globalPositionUpdate.returnOrientation())) * 1.1, Math.cos(Math.toRadians(globalPositionUpdate.returnOrientation())) * 1.1, Math.cos(Math.toRadians(globalPositionUpdate.returnOrientation())) * 1.1);
                telemetry.addData("X Position: %.2f", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Y Position: %.2f", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Orientation (Degrees): %.2f", globalPositionUpdate.returnOrientation());
                telemetry.addData("Thread Active", positionThread.isAlive());
                telemetry.addData("IMU", -robot.imu.getAngularOrientation().firstAngle);
                telemetry.addData("Vertical Left Position", -robot.verticalLeft.getCurrentPosition());
                telemetry.addData("Vertical Right Position", -robot.verticalRight.getCurrentPosition());
                telemetry.addData("Horizontal Position", -robot.horizontal.getCurrentPosition());
                telemetry.addData("change: ", globalPositionUpdate.changeInRobotOrientation);
                telemetry.addData("diff: ", globalPositionUpdate.verticalLeftEncoderWheelPosition - globalPositionUpdate.verticalRightEncoderWheelPosition);
                telemetry.update();
            } while (globalPositionUpdate.returnOrientation() < 90);
            setPowerAll(0,0,0,0);
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position: %.2f", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position: %.2f", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees): %.2f", globalPositionUpdate.returnOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.addData("IMU", -robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Vertical Left Position", -robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", -robot.verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", -robot.horizontal.getCurrentPosition());
            telemetry.addData("change: ", globalPositionUpdate.changeInRobotOrientation);
            telemetry.addData("diff: ", globalPositionUpdate.verticalLeftEncoderWheelPosition - globalPositionUpdate.verticalRightEncoderWheelPosition);
            telemetry.update();

            controllerInputA.update();
            controllerInputB.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }

    private void setPowerAll(double rf, double rb, double lf, double lb){
        robot.frontRightWheel.setPower(rf);
        robot.backRightWheel.setPower(rb);
        robot.frontLeftWheel.setPower(lf);
        robot.backLeftWheel.setPower(lb);
    }
}

