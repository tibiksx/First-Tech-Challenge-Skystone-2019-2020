package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.computervision.CameraThread;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.subsystems.ClawSystem;
import org.firstinspires.ftc.teamcode.subsystems.FoundationSystem;
import org.firstinspires.ftc.teamcode.subsystems.LifterThread;
import org.firstinspires.ftc.teamcode.subsystems.PositioningSystem;
import org.firstinspires.ftc.teamcode.subsystems.SliderThreadPID;
import org.firstinspires.ftc.teamcode.utilities.FieldStats;
import org.firstinspires.ftc.teamcode.utilities.LifterMethods;
import org.firstinspires.ftc.teamcode.utilities.PIDController;
import org.firstinspires.ftc.teamcode.utilities.Utilities;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.utilities.Utilities.AngleWrap;

/**
 * Autonomous Program that completes all tasks in a record time
 * Masterclass Copyright on Francisc Czobor - mentor of RO017 TITANS
 *             - term origin: an expensive-as-fuck Web Dev Course that basically teaches you nothing
 *             - couldn't be more suggestive
 * Red.
 */

@Autonomous(name = "Red Proficient Masterclass", group = "Autonomous")
public class RedProficientMasterclass extends LinearOpMode {

    private Hardware robot = new Hardware();
    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    private LifterThread lifterThread = null;
    private SliderThreadPID sliderThreadPID = null;
    private CameraThread cameraThread = null;

    private ClawSystem clawSystem = null;
    private PositioningSystem positioningSystem = null;
    private FoundationSystem foundationSystem = null;

    private final double initialXCoordinate = Utilities.CM_TO_TICKS(336);
    private final double initialYCoordinate = Utilities.CM_TO_TICKS(98);
    private final double initialOrientationDegrees = -90;

    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double rotation;

    private PIDController powerPID;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initIMU();

        cameraThread = new CameraThread(robot.webcam);
        Thread cameraRunner = new Thread(cameraThread);
        cameraRunner.start();

        cameraThread.setState(Utilities.CAMERA_STATE.INIT);
        sleep(4000);
        cameraThread.setState(Utilities.CAMERA_STATE.STREAM);

        powerPID = new PIDController(0, 0, 0);

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, Utilities.TICKS_PER_INCH, 50);
        globalPositionUpdate.setInitialCoordinates(initialXCoordinate, initialYCoordinate, toRadians(initialOrientationDegrees));
        globalPositionUpdate.reverseRightEncoder();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        clawSystem = new ClawSystem(robot.claw, robot.flipper);
        clawSystem.initial();
        clawSystem.raiseFlipper();

        positioningSystem = new PositioningSystem(robot.posLeft, robot.posRight);
        positioningSystem.initial();

        foundationSystem = new FoundationSystem(robot.foundationLeft, robot.foundationRight);
        foundationSystem.detach();

        lifterThread = new LifterThread(robot.leftLifter, robot.rightLifter, robot.button);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();

        sliderThreadPID = new SliderThreadPID(robot.slider);
        Thread sliderThread = new Thread(sliderThreadPID);
        sliderThread.start();

        if (!CameraThread.isHealthy()) {
            cameraRunner = new Thread(cameraThread);
            cameraRunner.start();

            cameraThread.setState(Utilities.CAMERA_STATE.INIT);
            sleep(1000);
            cameraThread.setState(Utilities.CAMERA_STATE.STREAM);
        }

        waitForStart();

        cameraThread.setState(Utilities.CAMERA_STATE.DETECT);
        cameraThread.setState(Utilities.CAMERA_STATE.KILL);

        positioningSystem.detach();
        goToPositionPID(FieldStats.REDStones.getSkystone(2).x, FieldStats.REDStones.getSkystone(2).y, 0.8, globalPositionUpdate, robot, Utilities.MOVEMENT_PRECISION.MEDIUM);
        sliderThreadPID.setTicks(1300);
        sleep(150);
        clawSystem.lowerFlipper();
        sleep(150);
        goToPositionNoTurning(FieldStats.REDStones.getSkystone(2).x + 20.5, FieldStats.REDStones.getSkystone(2).y, 0.7, globalPositionUpdate, robot,false);
        sliderThreadPID.setTicks(700);
        sleep(100);
        positioningSystem.attach();
        goToPositionPID(FieldStats.REDStones.getSkystone(2).x + 20.5, 180, 1, globalPositionUpdate, robot, Utilities.MOVEMENT_PRECISION.LOW);
        goToPositionPID(FieldStats.REDStones.getSkystone(2).x + 25.5, FieldStats.REDFoundation.DEFAULT_placingPosition.y, 1, globalPositionUpdate, robot, Utilities.MOVEMENT_PRECISION.LOW);
        clawSystem.attach();
        positioningSystem.detach();
        sleep(300);
        lifterThread.setTicks(LifterMethods.getTicksFromState(LifterMethods.LIFTER.FIRST));
        sleep(150);
        goToPositionPID(FieldStats.REDFoundation.DEFAULT_placingPosition.x - 30, FieldStats.REDFoundation.DEFAULT_placingPosition.y, 0.7, globalPositionUpdate, robot, Utilities.MOVEMENT_PRECISION.LOW);
        foundationSystem.attach();
        sleep(500);
        sliderThreadPID.setTicks(1800);
        while (getZAngle() < 87 && opModeIsActive()) {
            robot.frontLeftWheel.setPower(0.0);
            robot.backLeftWheel.setPower(0.0);
            robot.frontRightWheel.setPower(-1);
            robot.backRightWheel.setPower(-1);
        }
        clawSystem.detach();
        foundationSystem.detach();
        robot.frontLeftWheel.setPower(0.0);
        robot.backLeftWheel.setPower(0.0);
        robot.frontRightWheel.setPower(0.0);
        robot.backRightWheel.setPower(0.0);


//        goToPositionNoTurningWithTuning(FieldStats.REDStones.getSkystone(2).x, FieldStats.REDStones.getSkystone(2).y, 0.4, globalPositionUpdate, robot);
//        positioningSystem.detach();
//        sliderThreadPID.setTicks(1400);
//        sleep(300);
//        clawSystem.lowerFlipper();
//        sleep(350);
//        goToPositionNoTurning(FieldStats.REDStones.getSkystone(2).x + 17.4, FieldStats.REDStones.getSkystone(2).y, 0.7, globalPositionUpdate, robot,false);
//        sliderThreadPID.setTicks(350);
//        sleep(300);
//        positioningSystem.attach();
//        goToPositionNoTurning(FieldStats.REDStones.getSkystone(2).x + 21, 180, 0.7, globalPositionUpdate, robot,true);
//        goToPositionNoTurning(FieldStats.REDFoundation.DEFAULT_placingPosition.x + 15, FieldStats.REDFoundation.DEFAULT_placingPosition.y, 0.7, globalPositionUpdate, robot,false);
//        positioningSystem.detach();
//        clawSystem.attach();
//        sleep(250);
//        lifterThread.setTicks(LifterMethods.getTicksFromState(LifterMethods.LIFTER.FIRST));
//        sleep(100);
//        sliderThreadPID.setTicks(1200);
//        goToPositionNoTurning(FieldStats.REDFoundation.DEFAULT_placingPosition.x - 20, FieldStats.REDFoundation.DEFAULT_placingPosition.y, 0.7, globalPositionUpdate, robot,false);
//        foundationSystem.attach();
//        clawSystem.detach();
//        sleep(350);

    }

    private double movementRegression(double power) {
        return 1.593183 * (power * power * power) - 4.229217 * (power * power) + 3.635241 * power - 0.001638;
        // 0.19 - 1.055 + 1.8 - 0.001
        // 0.012 - 0.16 + 0.72 - 0.001
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
        } else {    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.frontLeftWheel.setPower(-power);
                robot.backLeftWheel.setPower(-power);
                robot.frontRightWheel.setPower(power);
                robot.backRightWheel.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }

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

    private void goToPositionPID(double x, double y, double moveSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot, Utilities.MOVEMENT_PRECISION movementPrecision) {
        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = Math.hypot(x - worldXPosition, y - worldYPosition);

        //PID controller that outputs a fraction of the power to apply to the motors
        //based on the distance to the target

        double p = Math.abs(moveSpeed/initialDistance) + 0.005;
        double i = p / 100.0;
        powerPID.setPID(p, i, 0);

        powerPID.setSetpoint(initialDistance);
        powerPID.setInputRange(0, initialDistance);
        powerPID.setOutputRange(0, moveSpeed);
        powerPID.setTolerance(5);
        powerPID.enable();

        do {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            worldAngle_rad = convertToUnitCircle(Math.toRadians(getZAngle() + initialOrientationDegrees));

            double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

            double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
            double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            movementXPower = Range.clip(movementXPower, -1, 1);
            movementYPower = Range.clip(movementYPower, -1, 1);


            double powerFraction = powerPID.performPID(initialDistance - distanceToTarget);
            if (powerFraction <= 0.25) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                powerPID.reset();
                break;
            }

            double _movement_x = movementXPower * powerFraction;
            double _movement_y = movementYPower * powerFraction;

            double drive = _movement_y;
            double strafe = -_movement_x;
            double turn = 0;

            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

            robot.frontLeftWheel.setPower(leftFrontPower);
            robot.frontRightWheel.setPower(rightFrontPower);
            robot.backLeftWheel.setPower(leftBackPower);
            robot.backRightWheel.setPower(rightBackPower);

            telemetry.addData(" Dist to target", distanceToTarget);
            telemetry.addData("PID power fraction", powerFraction);
            telemetry.addData("Error", powerPID.getError());
            telemetry.addData("On target:", powerPID.onTarget());
            telemetry.addData("Constants: ", powerPID.getP() + " " + powerPID.getI());
            telemetry.addData("Set Point", powerPID.getSetpoint());
            telemetry.update();
        } while (opModeIsActive() && !powerPID.onTarget());

        telemetry.addData("Finished","lol");
        telemetry.update();

        robot.frontLeftWheel.setPower(0);
        robot.frontRightWheel.setPower(0);
        robot.backLeftWheel.setPower(0);
        robot.backRightWheel.setPower(0);

    }

    private void goToX(double x, double y, double moveSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot) {
        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = x - worldXPosition;

        while (opModeIsActive()) {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            //worldAngle_rad = convertToUnitCircle(globalPositionUpdate.robotOrientationRadians);
            worldAngle_rad = convertToUnitCircle(Math.toRadians(getAngle() + initialOrientationDegrees));

            if (worldXPosition == x && worldYPosition == y) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                break;
            }

            double distanceToTarget = x - worldXPosition;

            double movementCoeff = movementRegression(Range.scale(distanceToTarget, 0.0, initialDistance, 0.0, 1.0));
            telemetry.addData("Power coeff ", movementCoeff);
            telemetry.update();

            if (movementCoeff < 0.25) {
                break;
            }

            double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
            double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            movementXPower = Range.clip(movementXPower, -1, 1);
            movementYPower = Range.clip(movementYPower, -1, 1);

            double _movement_x = movementXPower;
            double _movement_y = movementYPower;

//            if (initialDistance - distanceToTarget < 15 && moveSpeed > 0.8) {
//                _movement_x *= 0.8;
//                _movement_y *= 0.8;
//            }
//
//            if (distanceToTarget < 20 && moveSpeed > 0.3) {
//                _movement_x = _movement_x * 0.5;
//                _movement_y = _movement_y * 0.5;
//            } else if (distanceToTarget < 10 && moveSpeed > 0.8) {
//                _movement_x = _movement_x * 0.3;
//                _movement_y = _movement_y * 0.3;
//            }

            double drive = -_movement_y;
            double strafe = 0;
            double turn = 0;

            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

            if ((distanceToTarget < 5) || (_movement_x == 0 && _movement_y == 0)) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                break;
            }

            robot.frontLeftWheel.setPower(leftFrontPower * movementCoeff);
            robot.frontRightWheel.setPower(rightFrontPower * movementCoeff);
            robot.backLeftWheel.setPower(leftBackPower * movementCoeff);
            robot.backRightWheel.setPower(rightBackPower * movementCoeff);

//            telemetry.addData(" Dist to target", distanceToTarget);
//            telemetry.addData("Angle:", globalPositionUpdate.robotOrientationDeg);
//            telemetry.addData("X pos:", worldXPosition);
//            telemetry.addData("Y pos: ", worldYPosition);
//            telemetry.update();
        }

    }

    private void goToY(double x, double y, double moveSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot) {
        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = y - worldYPosition;

        while (opModeIsActive()) {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            //worldAngle_rad = convertToUnitCircle(globalPositionUpdate.robotOrientationRadians);
            worldAngle_rad = convertToUnitCircle(Math.toRadians(getAngle() + initialOrientationDegrees));

            if (worldXPosition == x && worldYPosition == y) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                break;
            }

            double distanceToTarget = y - worldYPosition;

            double movementCoeff = movementRegression(Range.scale(distanceToTarget, 0.0, initialDistance, 0.0, 1.0));
            telemetry.addData("Power coeff ", movementCoeff);
            telemetry.update();

            if (movementCoeff < 0.25) {
                break;
            }

            double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
            double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            movementXPower = Range.clip(movementXPower, -1, 1);
            movementYPower = Range.clip(movementYPower, -1, 1);

            double _movement_x = movementXPower;
            double _movement_y = movementYPower;

//            if (initialDistance - distanceToTarget < 15 && moveSpeed > 0.8) {
//                _movement_x *= 0.8;
//                _movement_y *= 0.8;
//            }
//
//            if (distanceToTarget < 20 && moveSpeed > 0.3) {
//                _movement_x = _movement_x * 0.5;
//                _movement_y = _movement_y * 0.5;
//            } else if (distanceToTarget < 10 && moveSpeed > 0.8) {
//                _movement_x = _movement_x * 0.3;
//                _movement_y = _movement_y * 0.3;
//            }

            double drive = 0;
            double strafe = -_movement_x;
            double turn = 0;

            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

            if ((distanceToTarget < 5) || (_movement_x == 0 && _movement_y == 0)) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                break;
            }

            robot.frontLeftWheel.setPower(leftFrontPower * movementCoeff);
            robot.frontRightWheel.setPower(rightFrontPower * movementCoeff);
            robot.backLeftWheel.setPower(leftBackPower * movementCoeff);
            robot.backRightWheel.setPower(rightBackPower * movementCoeff);

//            telemetry.addData(" Dist to target", distanceToTarget);
//            telemetry.addData("Angle:", globalPositionUpdate.robotOrientationDeg);
//            telemetry.addData("X pos:", worldXPosition);
//            telemetry.addData("Y pos: ", worldYPosition);
//            telemetry.update();
        }

    }

    public void goToPositionNoTurning(double x, double y, double moveSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot, boolean joint) {
        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = Math.hypot(x - worldXPosition, y - worldYPosition);

        while (opModeIsActive()) {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            //worldAngle_rad = convertToUnitCircle(globalPositionUpdate.robotOrientationRadians);
            worldAngle_rad = convertToUnitCircle(Math.toRadians(getAngle() + initialOrientationDegrees));

            if (worldXPosition == x && worldYPosition == y) {
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


            double _movement_x = movementXPower * moveSpeed;
            double _movement_y = movementYPower * moveSpeed;

            if (initialDistance - distanceToTarget < 15 && moveSpeed > 0.8) {
                _movement_x *= 0.8;
                _movement_y *= 0.8;
            }

            if(!joint) {
                if (distanceToTarget < 20 && moveSpeed > 0.3) {
                    _movement_x = _movement_x * 0.5;
                    _movement_y = _movement_y * 0.5;
                } else if (distanceToTarget < 10 && moveSpeed > 0.8) {
                    _movement_x = _movement_x * 0.3;
                    _movement_y = _movement_y * 0.3;
                }
            }

            double drive = _movement_y;
            double strafe = -_movement_x;
            double turn = 0;

            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

            if ((distanceToTarget < 5) || (_movement_x == 0 && _movement_y == 0)) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                break;
            }


            robot.frontLeftWheel.setPower(leftFrontPower);
            robot.frontRightWheel.setPower(rightFrontPower);
            robot.backLeftWheel.setPower(leftBackPower);
            robot.backRightWheel.setPower(rightBackPower);

            telemetry.addData(" Dist to target", distanceToTarget);
            telemetry.addData("Angle:", globalPositionUpdate.robotOrientationDeg);
            telemetry.addData("X pos:", worldXPosition);
            telemetry.addData("Y pos: ", worldYPosition);
            telemetry.update();
        }

    }

    private void goToPositionNoTurningWithTuning(double x, double y, double moveSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot) {
        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = Math.hypot(x - worldXPosition, y - worldYPosition);

        while (opModeIsActive()) {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            //worldAngle_rad = convertToUnitCircle(globalPositionUpdate.robotOrientationRadians);
            worldAngle_rad = convertToUnitCircle(Math.toRadians(getAngle() + initialOrientationDegrees));

            if (worldXPosition == x && worldYPosition == y) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                break;
            }

            double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

            double movementCoeff = movementRegression(Range.scale(distanceToTarget, 0.0, initialDistance, 0.0, 1.0));
            telemetry.addData("Power coeff ", movementCoeff);
            telemetry.update();

            double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
            double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            movementXPower = Range.clip(movementXPower, -1, 1);
            movementYPower = Range.clip(movementYPower, -1, 1);

            double _movement_x = movementXPower;
            double _movement_y = movementYPower;

//            if (initialDistance - distanceToTarget < 15 && moveSpeed > 0.8) {
//                _movement_x *= 0.8;
//                _movement_y *= 0.8;
//            }
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

            if ((distanceToTarget < 5) || (_movement_x == 0 && _movement_y == 0)) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                break;
            }

            robot.frontLeftWheel.setPower(leftFrontPower * movementCoeff);
            robot.frontRightWheel.setPower(rightFrontPower * movementCoeff);
            robot.backLeftWheel.setPower(leftBackPower * movementCoeff);
            robot.backRightWheel.setPower(rightBackPower * movementCoeff);

//            telemetry.addData(" Dist to target", distanceToTarget);
//            telemetry.addData("Angle:", globalPositionUpdate.robotOrientationDeg);
//            telemetry.addData("X pos:", worldXPosition);
//            telemetry.addData("Y pos: ", worldYPosition);
//            telemetry.update();
        }

    }

    public void goToPositionTurning(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot, Telemetry telemetry) {

        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = Math.hypot(x - worldXPosition, y - worldYPosition);

        if (preferredAngle <= TWO_PI) preferredAngle += TWO_PI;
        if (preferredAngle >= TWO_PI) preferredAngle -= TWO_PI;
        preferredAngle = convertToUnitCircle(preferredAngle);

        while (opModeIsActive()) {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            //worldAngle_rad = convertToUnitCircle(globalPositionUpdate.robotOrientationRadians);
            worldAngle_rad = convertToUnitCircle(Math.toRadians(getAngle() + initialOrientationDegrees));

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

            double _movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
            double _movement_x = movementXPower * movementSpeed;
            double _movement_y = movementYPower * movementSpeed;

            if (initialDistance - distanceToTarget < 5) {
                _movement_x *= 0.7;
                _movement_y *= 0.7;
            }

            if (abs(relativeTurnAngle) < Math.toRadians(15)) _movement_turn *= 0.3;

            if (distanceToTarget < 20 && distanceToTarget > 10 && movementSpeed > 0.5) {
                _movement_x = _movement_x * 0.5;
                _movement_y = _movement_y * 0.5;
            }
            if (distanceToTarget < 10 && movementSpeed > 0.5) {
                _movement_x = _movement_x * 0.3;
                _movement_y = _movement_y * 0.3;
            }

            double drive = _movement_y;
            double strafe = -_movement_x;
            double turn = -_movement_turn;

            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

            if ((distanceToTarget < 5) || (_movement_x == 0 && _movement_y == 0 && _movement_turn == 0)) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                break;
            }


            robot.frontLeftWheel.setPower(leftFrontPower);
            robot.frontRightWheel.setPower(rightFrontPower);
            robot.backLeftWheel.setPower(leftBackPower);
            robot.backRightWheel.setPower(rightBackPower);

            telemetry.addData(" Dist to target", distanceToTarget);
            telemetry.addData("Angle:", Math.toDegrees(worldAngle_rad));
            telemetry.addData("X pos:", worldXPosition);
            telemetry.addData("Y pos: ", worldYPosition);
            telemetry.update();

        }

    }


    final static double TWO_PI = 2 * PI;
    final static double HALF_PI = PI / 2;

    private double convertToUnitCircle(double angle) {
        //reduce back to unit circle
        if (angle > TWO_PI) angle -= TWO_PI;
        if (angle < 0) angle += TWO_PI;

        ///////// FIRST QUADRANT /////////////

        if (angle >= 0 && angle < HALF_PI) {

            angle = abs(angle - HALF_PI);

            if (angle > TWO_PI) angle -= TWO_PI;

            return angle;
        }

        ////////// SECOND QUADRANT ////////////

        if (angle >= HALF_PI && angle <= PI) {

            angle = 3 * HALF_PI - (angle + PI - TWO_PI);

            if (angle > TWO_PI) angle -= TWO_PI;

            return angle;
        }

        //////////// THIRD QUADRANT ///////////

        if (angle > PI && angle <= 3 * HALF_PI) {

            angle = 3 * HALF_PI + (PI - angle);

            if (angle > TWO_PI) angle -= TWO_PI;

            return angle;
        }

        /////////// FOURTH QUADRANT ////////////

        if (angle > 3 * HALF_PI && angle <= TWO_PI) {

            angle = PI - (angle + PI) + TWO_PI + HALF_PI;

            if (angle > TWO_PI) angle -= TWO_PI;

            return angle;
        }
        return angle;
    }


}
