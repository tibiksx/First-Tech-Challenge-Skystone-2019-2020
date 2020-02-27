package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_RED;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

public class Utilities {

    public enum MOVEMENT_PRECISION {
        LOW,
        MEDIUM,
        HIGH,
        STANDARD
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


    public static double AngleWrap(double angle) {
        while (angle < -PI)
            angle += 2 * PI;

        while (angle > PI)
            angle -= 2 * PI;

        return angle;
    }


}
