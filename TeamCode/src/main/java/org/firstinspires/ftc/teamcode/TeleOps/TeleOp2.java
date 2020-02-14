package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Misc.Utilities;
import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import java.text.DecimalFormat;
import java.text.NumberFormat;

@TeleOp(name = "Test TeleOp 2")
public class TeleOp2 extends Robot {

    private static double xGoal = 0;
    private static double yGoal = 60;
    private static double turnSpeed = 0.7;
    private static final double preferredAngle = Math.toRadians(0);

    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void init() {
        super.init();

        robot.init(hardwareMap);

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, Utilities.TICKS_PER_INCH, 75);

        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();

    }

    @Override
    public void loop() {

//        float drive, turn, strafe;
//
//        drive = -gamepad1.left_stick_y;
//        strafe = -gamepad1.left_stick_x;
//        turn = gamepad1.right_stick_x;
//
//        double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
//        double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
//        double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
//        double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
//
//        robot.frontLeftWheel.setPower(leftFrontPower);
//        robot.frontRightWheel.setPower(rightFrontPower);
//        robot.backLeftWheel.setPower(leftBackPower);
//        robot.backRightWheel.setPower(rightBackPower);

        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad = globalPositionUpdate.robotOrientationRadians;

        double distToTarget = Math.hypot(xGoal - worldXPosition, yGoal - worldYPosition);
        double absoluteAngleToTarget = Math.atan2(yGoal - worldYPosition, xGoal - worldXPosition);


        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - worldAngle_rad);

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double relativeTurnAngle = preferredAngle - worldAngle_rad;

        double movementTurn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1);

        double d = movementYPower;
        double t = movementTurn;
        double s = -movementXPower;
        double fl = d + t - s;
        double bl = d + t + s;
        double fr = d - t + s;
        double br = d - t - s;

        double powerFraction = 1;//Range.scale(distToTarget,0,initialDistance,0,1);

        if(distToTarget < 30)  {
            powerFraction = 0.5;
        }

        if(distToTarget < 10) {
            powerFraction = 0;
        }

        NumberFormat formatter = new DecimalFormat("#0.000");
        telemetry.log().clear();
        telemetry.addData("X: ", formatter.format(worldXPosition));
        telemetry.addData("Y: ", formatter.format(worldYPosition));
        telemetry.addData("Angle: ", formatter.format(Math.toDegrees(worldAngle_rad)));
        telemetry.addData("Goal: ", formatter.format(xGoal) + " " + formatter.format(yGoal));
        telemetry.addData("Distance to target: ", formatter.format(distToTarget));
        telemetry.addData("Abs Angle To Target: ", formatter.format(Math.toDegrees(absoluteAngleToTarget)));
        telemetry.addData("Relative Angle To Point: ", formatter.format(Math.toDegrees(relativeAngleToPoint)));
        telemetry.addData("Relative X To Point: ", formatter.format(relativeXToPoint));
        telemetry.addData("Relative Y To Point: ", formatter.format(relativeYToPoint));
        telemetry.addData("Relative Turn Angle: ",formatter.format(Math.toDegrees(relativeTurnAngle)));

        telemetry.addData("Movement X Power: ",(movementXPower));
        telemetry.addData("Movement Y Power: ", (movementYPower));
        telemetry.addData("Movement Turn: ", (movementTurn));
        telemetry.addData( "POWERS ",formatter.format(fl) + " " + formatter.format(bl) + " " + formatter.format(fr) + " " + formatter.format(br));
        telemetry.update();

//        robot.frontLeftWheel.setPower(fl * powerFraction);
//        robot.frontRightWheel.setPower(fr * powerFraction);
//        robot.backLeftWheel.setPower(bl * powerFraction);
//        robot.backRightWheel.setPower(br * powerFraction);
    }

    private double AngleWrap(double angle) {
        while (angle < -Math.PI)
            angle += 2 * Math.PI;

        while (angle > Math.PI)
            angle -= 2 * Math.PI;

        return angle;
    }


}
