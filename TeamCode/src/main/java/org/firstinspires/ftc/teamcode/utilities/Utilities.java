package org.firstinspires.ftc.teamcode.utilities;

import android.content.Context;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

import java.io.File;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

public class Utilities {

    public enum MOVEMENT_PRECISION {
        LOW,
        MEDIUM,
        HIGH,
        STANDARD
    }

    public enum CAMERA_STATE {
        NULL,
        INIT,
        STREAM,
        DETECT,
        KILL
    }

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



    public static void goToPositionWIP(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot, Telemetry telemetry) {

        double worldXPosition;
        double worldYPosition;
        double worldAngle_rad;

        preferredAngle = convertToUnitCircle(preferredAngle);

        while(true) {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            worldAngle_rad = convertToUnitCircle(globalPositionUpdate.robotOrientationRadians);

            if(worldXPosition == x && worldYPosition == y && worldAngle_rad == preferredAngle) break;

            double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

            double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
            double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            movementXPower = Range.clip(movementXPower, -1, 1);
            movementYPower = Range.clip(movementYPower, -1, 1);

            //double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;  //i don't fucking understand this line so i used my own

            double relativeTurnAngle = preferredAngle - worldAngle_rad;

            double _movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
            double _movement_x = movementXPower * movementSpeed;
            double _movement_y = movementYPower * movementSpeed;

            if(distanceToTarget < 30){
                _movement_x = _movement_x * 0.5;
                _movement_y = _movement_y * 0.5;
            }
            if (distanceToTarget < 10) _movement_turn = 0;

            if (distanceToTarget < 7 && _movement_turn == 0) {
                _movement_x = _movement_y = 0;
            }

            double drive = _movement_y;
            double strafe = -_movement_x;
            double turn = _movement_turn;

            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

            if(  (distanceToTarget < 5) || (_movement_x == 0 && _movement_y == 0 && _movement_turn == 0) ) break;

            robot.frontLeftWheel.setPower(leftFrontPower);
            robot.frontRightWheel.setPower(rightFrontPower);
            robot.backLeftWheel.setPower(leftBackPower);
            robot.backRightWheel.setPower(rightBackPower);

            telemetry.addData(" Dist to trgt",distanceToTarget);
            telemetry.addData("absoluteAngleToTarget: ",Math.toDegrees(absoluteAngleToTarget));
            telemetry.addData("relativeAngleToPoint",Math.toDegrees(relativeAngleToPoint));
            telemetry.addData("relativeXToPoint",relativeXToPoint);
            telemetry.addData("relativeYToPoint",relativeYToPoint);
            telemetry.addData("relativeTurnAngle",Math.toDegrees(relativeTurnAngle));
            telemetry.update();


        }

    }

    public static double AngleWrap(double angle) {
        while (angle < -PI)
            angle += 2 * PI;

        while (angle > PI)
            angle -= 2 * PI;

        return angle;
    }


    final static double TWO_PI = 2 * PI;
    final static double HALF_PI = PI / 2;

    private static double convertToUnitCircle(double angle) {
        //reduce back to unit circle
        if(angle > TWO_PI) angle -= TWO_PI;
        if(angle >= 0 && angle < HALF_PI) {  //first quadrant
            angle =  abs(angle - HALF_PI);
            if(angle > TWO_PI) angle -= TWO_PI;
            return angle;
        }

        if(angle >= HALF_PI && angle <=PI)   //second quadrant
        {
            angle = 3*HALF_PI -(angle + PI - TWO_PI);

            if(angle > TWO_PI) angle -= TWO_PI;
            return angle;
        }

        if(angle > PI && angle <= 3*HALF_PI) //third quadrant
        {
            angle =  3*HALF_PI + (PI - angle);
            if(angle > TWO_PI) angle -= TWO_PI;
            return angle;
        }

        if(angle > 3*HALF_PI && angle <=TWO_PI) //fourth quadrant
        {
            angle = PI - (angle + PI) + TWO_PI + HALF_PI;
            if(angle > TWO_PI) angle -= TWO_PI;
            return angle;
        }
        return angle;
    }

    public static void deleteCache(Context context) {
        try {
            File dir = context.getCacheDir();
            deleteDir(dir);
        } catch (Exception e) { e.printStackTrace();}
    }

    public static boolean deleteDir(File dir) {
        if (dir != null && dir.isDirectory()) {
            String[] children = dir.list();
            for (int i = 0; i < children.length; i++) {
                boolean success = deleteDir(new File(dir, children[i]));
                if (!success) {
                    return false;
                }
            }
            return dir.delete();
        } else if(dir!= null && dir.isFile()) {
            return dir.delete();
        } else {
            return false;
        }
    }
}
