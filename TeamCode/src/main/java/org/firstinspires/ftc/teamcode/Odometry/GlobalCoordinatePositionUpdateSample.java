package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Misc.Constants;
import org.openftc.revextensions2.ExpansionHubMotor;


/**
 * Created by Sarthak on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */
@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
public class GlobalCoordinatePositionUpdateSample extends LinearOpMode {

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = Constants.TICKS_PER_INCH;

    private Hardware robot = new Hardware();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */
//        robot.leftEncoderMotor.setDirection(ExpansionHubMotor.Direction.REVERSE);
//        robot.rightEncoderMotor.setDirection(ExpansionHubMotor.Direction.REVERSE);
//        robot.backEncoderMotor.setDirection(ExpansionHubMotor.Direction.REVERSE);

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        robot.rightEncoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftEncoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backEncoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        /*
          *****************
          OpMode Begins Here
          *****************
         */

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.leftEncoderMotor, robot.rightEncoderMotor, robot.backEncoderMotor, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());

            telemetry.addData("left encoder @", robot.leftEncoderMotor.getCurrentPosition());
            telemetry.addData("right encoder @", robot.rightEncoderMotor.getCurrentPosition());
            telemetry.addData("back encoder @", robot.backEncoderMotor.getCurrentPosition());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }
}