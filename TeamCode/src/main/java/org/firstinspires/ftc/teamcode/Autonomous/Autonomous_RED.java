package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ComputerVision.CameraThread;
import org.firstinspires.ftc.teamcode.ComputerVision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.FieldStats;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Misc.*;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;
import org.firstinspires.ftc.teamcode.Threads.SliderThreadPID;
import org.opencv.core.Point;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Misc.Utilities.AngleWrap;
import static org.firstinspires.ftc.teamcode.Misc.Utilities.deleteCache;

@Autonomous(name = "Auto RED", group = "Autonomous")
public class Autonomous_RED extends LinearOpMode {

    private Hardware robot = new Hardware();
    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    private LifterThread lifterThread = null;
    private CameraThread cameraThread = null;
    private SliderThreadPID sliderThreadPID;

    private ClawSystem clawSystem = null;
    private PositioningSystem positioningSystem = null;
    private FoundationSystem foundationSystem = null;

//    private SkystoneDetector skystoneDetector;

    private final double initialXCoordinate = Utilities.CM_TO_TICKS(336);
    private final double initialYCoordinate = Utilities.CM_TO_TICKS(98);
    private final double initialOrientationDegrees = -90;

    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double rotation;


    @Override
    public void runOpMode() throws InterruptedException {

        deleteCache(AppUtil.getDefContext()); //delete cache

        robot.init(hardwareMap);
        robot.initIMU();
        robot.initWebcam();

        FieldStats.REDStones.unMarkAllStones();

        cameraThread = new CameraThread(robot.webcam);
        Thread cameraRunner = new Thread(cameraThread);
        cameraRunner.start();

        cameraThread.setState(Utilities.CAMERA_STATE.INIT);
        sleep(2000);
        cameraThread.setState(Utilities.CAMERA_STATE.STREAM);

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, robot.ExpansionHub1, Utilities.TICKS_PER_INCH, 50);
        globalPositionUpdate.setInitialCoordinates(initialXCoordinate, initialYCoordinate, toRadians(initialOrientationDegrees));
        globalPositionUpdate.reverseRightEncoder();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        clawSystem = new ClawSystem(robot.claw, robot.flipper);
        clawSystem.Initial();
        clawSystem.raiseFlipper();

        positioningSystem = new PositioningSystem(robot.posLeft, robot.posRight);
        positioningSystem.Initial();

        foundationSystem = new FoundationSystem(robot.foundationLeft, robot.foundationRight);
        foundationSystem.Detach();


        lifterThread = new LifterThread(robot.leftLifter, robot.rightLifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();

        sliderThreadPID = new SliderThreadPID(robot.slider);
        Thread sliderThread = new Thread(sliderThreadPID);
        sliderThread.start();

        if (!CameraThread.isAlive()) {
            cameraRunner = new Thread(cameraThread);
            cameraRunner.start();

            cameraThread.setState(Utilities.CAMERA_STATE.INIT);
            sleep(1000);
            cameraThread.setState(Utilities.CAMERA_STATE.STREAM);
        }


        waitForStart();

        cameraThread.setState(Utilities.CAMERA_STATE.DETECT);
        sleep(100);
        cameraThread.setState(Utilities.CAMERA_STATE.KILL);
        cameraRunner.join();

        //find the closest stone to pick up
        FieldStats.Stone closestStone1 = new FieldStats.Stone(new Point(-1, -1), false);
        int closestStoneIndex = -1;
        for (int i = 6; i >= 1; i--) {
            if (FieldStats.REDStones.stoneArray[i].isSkystone()) {
                closestStone1 = FieldStats.REDStones.stoneArray[i];
                closestStoneIndex = i;
                break;
            }
        }
        if (closestStone1.getPosition().x != -1) {
            positioningSystem.Detach();
            sliderThreadPID.setTicks(1000);

            goToPositionNoTurning(closestStone1.getPosition(), 1, globalPositionUpdate, robot, new PIDController(0.02, 0.0002, 0),5);
            clawSystem.lowerFlipper();
            sleep(600);
            sliderThreadPID.setTicks(600);

            telemetry.addData("Target: ",closestStone1.getPosition().x + "  " + closestStone1.getPosition().y);
            telemetry.addData("index:",closestStoneIndex);
            telemetry.addData("X Y",Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition)+ "  "
                    +Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition));
            telemetry.update();
            sleep(5000);


        } else {  //if we did not find a stone for some reason just park for now
            goToPositionNoTurning(new Point(285, 180), 1, globalPositionUpdate, robot, new PIDController(0, 0, 0),5);
        }
    }

    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void rotate(int degrees, double power, PIDController pidRotate, Hardware robot) {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                robot.frontLeftWheel.setPower(power);
                robot.backLeftWheel.setPower(power);
                robot.frontRightWheel.setPower(-power);
                robot.backRightWheel.setPower(-power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.frontLeftWheel.setPower(-power);
                robot.backLeftWheel.setPower(-power);
                robot.frontRightWheel.setPower(power);
                robot.backRightWheel.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.frontLeftWheel.setPower(-power);
                robot.backLeftWheel.setPower(-power);
                robot.frontRightWheel.setPower(power);
                robot.backRightWheel.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.frontLeftWheel.setPower(0);
        robot.backLeftWheel.setPower(0);
        robot.frontRightWheel.setPower(0);
        robot.backRightWheel.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private double getZAngle() {
        return (-robot.imu.getAngularOrientation().firstAngle);
    }


    private void goToPositionNoTurning(Point target, double moveSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot, PIDController pidController, int tolerance) {
        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = Math.hypot(target.x - worldXPosition, target.y - worldYPosition);

        //PID controller that outputs a fraction of the power to apply to the motors

        pidController.setSetpoint(initialDistance);
        pidController.setInputRange(0, initialDistance);
        pidController.setOutputRange(0, moveSpeed);
        pidController.setTolerance(tolerance);
        pidController.enable();

        do {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            worldAngle_rad = covertToUnitCircle(Math.toRadians(getZAngle() + initialOrientationDegrees));

            double distanceToTarget = Math.hypot(target.x - worldXPosition, target.y - worldYPosition);

            double absoluteAngleToTarget = Math.atan2(target.y - worldYPosition, target.x - worldXPosition);
            double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            movementXPower = Range.clip(movementXPower, -1, 1);
            movementYPower = Range.clip(movementYPower, -1, 1);


            double powerFraction = pidController.performPID(initialDistance - distanceToTarget);

            if( initialDistance - distanceToTarget < 10 && initialDistance > 15) powerFraction = 0.6;
//            if (powerFraction <= 0.25) {
//                robot.frontLeftWheel.setPower(0);
//                robot.frontRightWheel.setPower(0);
//                robot.backLeftWheel.setPower(0);
//                robot.backRightWheel.setPower(0);
//                break;
//            }


            double _movement_x = movementXPower * powerFraction;
            double _movement_y = movementYPower * powerFraction;


//
//            if (distanceToTarget < 20 && moveSpeed > 0.3) {
//                _movement_x = _movement_x * 0.5;
//                _movement_y = _movement_y * 0.5;
//            } else if (distanceToTarget < 10 && moveSpeed > 0.8) {
//                _movement_x = _movement_x * 0.3;
//                _movement_y = _movement_y * 0.3;
//            }


            double drive = _movement_y;
            double strafe = -_movement_x;
            double turn = 0;

            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

//            if ((distanceToTarget < 5) || (_movement_x == 0 && _movement_y == 0)) {
//                robot.frontLeftWheel.setPower(0);
//                robot.frontRightWheel.setPower(0);
//                robot.backLeftWheel.setPower(0);
//                robot.backRightWheel.setPower(0);
//                break;
//            }


            robot.frontLeftWheel.setPower(leftFrontPower);
            robot.frontRightWheel.setPower(rightFrontPower);
            robot.backLeftWheel.setPower(leftBackPower);
            robot.backRightWheel.setPower(rightBackPower);

            NumberFormat numberFormat = new DecimalFormat("#0.00");

            telemetry.addData(" Dist to target", distanceToTarget);
            telemetry.addData("PID power fraction", powerFraction);
            telemetry.addData("Error", pidController.getError());
            telemetry.addData("On target:", pidController.onTarget());
            telemetry.addData("Constants: ", numberFormat.format(pidController.getP()) + " " + numberFormat.format(pidController.getI()));
            telemetry.addData("Set Point", pidController.getSetpoint());
            //telemetry.addData("IMU angle", getZAngle());
            //telemetry.addData("Angle:", Math.toDegrees(worldAngle_rad));
            //telemetry.addData("X pos:", worldXPosition);
            //telemetry.addData("Y pos: ", worldYPosition);
            telemetry.update();
        } while (opModeIsActive() && !pidController.onTarget());

        telemetry.addData("Finished", "lol");
        telemetry.update();

        robot.frontLeftWheel.setPower(0);
        robot.frontRightWheel.setPower(0);
        robot.backLeftWheel.setPower(0);
        robot.backRightWheel.setPower(0);

    }


    public void goToPositionTurning(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot, Telemetry telemetry, PIDController pidController) {

        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = Math.hypot(x - worldXPosition, y - worldYPosition);

        if (preferredAngle <= TWO_PI) preferredAngle += TWO_PI;
        if (preferredAngle >= TWO_PI) preferredAngle -= TWO_PI;
        preferredAngle = covertToUnitCircle(preferredAngle);


        pidController.setSetpoint(initialDistance);
        pidController.setInputRange(0, initialDistance);
        pidController.setOutputRange(0, movementSpeed);
        pidController.setTolerance(5);
        pidController.enable();

        do {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            worldAngle_rad = covertToUnitCircle(Math.toRadians(getZAngle() + initialOrientationDegrees));

            if (worldXPosition == x && worldYPosition == y && worldAngle_rad == preferredAngle) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                break;
            }

            double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

            double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
            double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            movementXPower = Range.clip(movementXPower, -1, 1);
            movementYPower = Range.clip(movementYPower, -1, 1);

            if (preferredAngle >= TWO_PI) preferredAngle -= TWO_PI;
            double relativeTurnAngle = preferredAngle - worldAngle_rad;

            relativeTurnAngle = AngleWrap(relativeTurnAngle);

            double powerFraction = pidController.performPID(initialDistance - distanceToTarget);
            if (powerFraction <= 0.30) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                break;
            }


            double _movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
            double _movement_x = movementXPower * powerFraction;
            double _movement_y = movementYPower * powerFraction;

//            if (initialDistance - distanceToTarget < 5) {
//                _movement_x *= 0.7;
//                _movement_y *= 0.7;
//            }

//            if (abs(relativeTurnAngle) < Math.toRadians(15)) _movement_turn *= 0.3;
//
//            if (distanceToTarget < 20 && distanceToTarget > 10 && movementSpeed > 0.5) {
//                _movement_x = _movement_x * 0.5;
//                _movement_y = _movement_y * 0.5;
//            }
//            if (distanceToTarget < 10 && movementSpeed > 0.5) {
//                _movement_x = _movement_x * 0.3;
//                _movement_y = _movement_y * 0.3;
//            }


            double drive = _movement_y;
            double strafe = -_movement_x;
            double turn = -_movement_turn;

            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

//            if ((distanceToTarget < 5) || (_movement_x == 0 && _movement_y == 0 && _movement_turn == 0)) {
//                robot.frontLeftWheel.setPower(0);
//                robot.frontRightWheel.setPower(0);
//                robot.backLeftWheel.setPower(0);
//                robot.backRightWheel.setPower(0);
//                break;
//            }


            robot.frontLeftWheel.setPower(leftFrontPower);
            robot.frontRightWheel.setPower(rightFrontPower);
            robot.backLeftWheel.setPower(leftBackPower);
            robot.backRightWheel.setPower(rightBackPower);

            telemetry.addData(" Dist to target", distanceToTarget);
            telemetry.addData("Angle:", Math.toDegrees(worldAngle_rad));
            telemetry.addData("X pos:", worldXPosition);
            telemetry.addData("Y pos: ", worldYPosition);
            telemetry.update();

        } while (opModeIsActive() && !pidController.onTarget());

        robot.frontLeftWheel.setPower(0);
        robot.frontRightWheel.setPower(0);
        robot.backLeftWheel.setPower(0);
        robot.backRightWheel.setPower(0);
    }


    private final static double TWO_PI = 2 * PI;
    private final static double HALF_PI = PI / 2;

    private double covertToUnitCircle(double angle) {
        //reduce back to unit circle
        if (angle > TWO_PI) angle -= TWO_PI;
        if (angle < 0) angle += TWO_PI;
        if (angle >= 0 && angle < HALF_PI) {  //first quadrant
            angle = abs(angle - HALF_PI);
            if (angle > TWO_PI) angle -= TWO_PI;
            return angle;
        }

        if (angle >= HALF_PI && angle <= PI)   //second quadrant
        {
            angle = 3 * HALF_PI - (angle + PI - TWO_PI);

            if (angle > TWO_PI) angle -= TWO_PI;
            return angle;
        }

        if (angle > PI && angle <= 3 * HALF_PI) //third quadrant
        {
            angle = 3 * HALF_PI + (PI - angle);
            if (angle > TWO_PI) angle -= TWO_PI;
            return angle;
        }

        if (angle > 3 * HALF_PI && angle <= TWO_PI) //fourth quadrant
        {
            angle = PI - (angle + PI) + TWO_PI + HALF_PI;
            if (angle > TWO_PI) angle -= TWO_PI;
            return angle;
        }
        return angle;
    }

}
