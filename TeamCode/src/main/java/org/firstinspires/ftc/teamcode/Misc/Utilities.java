package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

public class Utilities {
    public final static double TICKS_PER_INCH = 259.84481;
    public final static double TICKS_PER_CM = 101.85916;
    public final static double TICKS_PER_REV = 1600;

    public static final double FIELD_LENGTH = 358.775;

    public static double CM_TO_INCH(double value) {
        return value / 2.54;
    }

    public static double INCH_TO_CM(double value) {
        return value * 2.54;
    }

    public static double CM_TO_TICKS(double value) {
        return value * TICKS_PER_CM;
    }

    public static double TICKS_TO_CM(double value) {
        return value / TICKS_PER_CM;
    }

    public static double INCH_TO_TICKS(double value) {
        return value * TICKS_PER_INCH;
    }

    public static double TICKS_TO_INCH(double value) {
        return value / TICKS_PER_INCH;
    }


    public static void goToPosition(double xGoal, double yGoal, double preferredAngle,double moveSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot) {
        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.returnXCoordinate());
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.returnYCoordinate());
        double worldAngle_rad;


        while((worldXPosition < xGoal && worldYPosition < yGoal) || (worldXPosition >xGoal && worldYPosition > yGoal)) {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.returnXCoordinate());
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.returnYCoordinate());
            worldAngle_rad = globalPositionUpdate.returnOrientationRad();
            double distToTarget = Math.hypot(xGoal - worldXPosition, yGoal - worldYPosition);
            double absoluteAngleToTarget = Math.atan2(yGoal - worldYPosition, xGoal - worldXPosition);
            double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distToTarget;

            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            movementXPower = Range.clip(movementXPower,-1,1);
            movementYPower = Range.clip(movementYPower, -1 , 1);

            double relativeTurnAngle = preferredAngle - worldAngle_rad;

            double movementTurn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1);

            double d = movementYPower;
            double t = 0;
            double s = -movementXPower;
            double fl = d + t - s;
            double bl = d + t + s;
            double fr = d - t + s;
            double br = d - t - s;

            double powerFraction = 1;

            if(distToTarget < 20)  {
                powerFraction = 0.5;
            }

            if(distToTarget < 3) {
                break;
            }

            robot.frontLeftWheel.setPower(fl * powerFraction * moveSpeed);
            robot.frontRightWheel.setPower(fr * powerFraction * moveSpeed);
            robot.backLeftWheel.setPower(bl * powerFraction * moveSpeed);
            robot.backRightWheel.setPower(br * powerFraction * moveSpeed);
        }

    }

    private static double AngleWrap(double angle) {
        while (angle < -Math.PI)
            angle += 2 * Math.PI;

        while (angle > Math.PI)
            angle -= 2 * Math.PI;

        return angle;
    }

}
