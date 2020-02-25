package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Misc.Utilities;
import org.firstinspires.ftc.teamcode.Misc.Robot;

/**
 * This class is test for odometry. It displays X, Y and Orientation in degrees.
 * It also displays the encoder positions.
 */

@TeleOp(name = "Odometry Test", group = "Pushbot")
public class OdometryTest extends Robot {

    private double powerCoeff = 1;

    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void init() {
        super.init();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, Utilities.TICKS_PER_INCH, 75);
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

        telemetry.log().clear();
        telemetry.addData("X:  ",Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition));
        telemetry.addData("Y: ", Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition));
        telemetry.addData("Angle: ",globalPositionUpdate.robotOrientationDeg);
        telemetry.addData("Left: ",robot.verticalLeft.getCurrentPosition());
        telemetry.addData("Right: ",robot.verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal: ",robot.horizontal.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }

}
