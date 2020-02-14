package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Utilities;
import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

/**
 * This class just includes the driving part of the robot, including
 * displaying the X,Y and orientation of the robot.
 */

@TeleOp(name = "Driving + Odometry", group = "Pushbot")
public class DrivingTeleOp extends Robot {

    private double powerCoeff = 1;

    private Telemetry.Item globalXCoordinateTelemetry = null;
    private Telemetry.Item globalYCoordinateTelemetry = null;
    private Telemetry.Item globalOrientationTelemetry = null;

    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void init() {
        super.init();

        globalXCoordinateTelemetry = telemetry.addData("X: ",0);
        globalYCoordinateTelemetry = telemetry.addData("Y: ",0);
        globalOrientationTelemetry = telemetry.addData("Angle: ",0);
        telemetry.update();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, Utilities.TICKS_PER_INCH, 75);

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        globalPositionUpdate.reverseRightEncoder();

        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
    }


    @Override
    public void loop() {
        super.loop();

        if (controllerInputA.leftBumperOnce() && powerCoeff >= 0.2) {
            powerCoeff -= 0.1;
        } else if (controllerInputA.rightBumperOnce() && powerCoeff <= 1) {
            powerCoeff += 0.1;
        }

        powerCoeff = Range.clip(powerCoeff, 0.0, 1.0);

        float drive, turn, strafe;

        drive = -gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

        robot.frontLeftWheel.setPower(powerCoeff * leftFrontPower);
        robot.frontRightWheel.setPower(powerCoeff * rightFrontPower);
        robot.backLeftWheel.setPower(powerCoeff * leftBackPower);
        robot.backRightWheel.setPower(powerCoeff * rightBackPower);

        globalXCoordinateTelemetry.setValue("%.3f", Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition));
        globalYCoordinateTelemetry.setValue("%.3f", Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition));
        globalOrientationTelemetry.setValue("%.3f",globalPositionUpdate.robotOrientationDeg);
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }

}
