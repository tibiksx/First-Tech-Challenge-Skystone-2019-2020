package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
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
import org.opencv.core.Point;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.utilities.Utilities.AngleWrap;
import static org.firstinspires.ftc.teamcode.utilities.Utilities.deleteCache;

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

        deleteCache(AppUtil.getDefContext());

        robot.init(hardwareMap);
        robot.initIMU();

        FieldStats.REDStones.unMarkAllStones();

        cameraThread = new CameraThread(robot.webcam);
        Thread cameraRunner = new Thread(cameraThread);
        cameraRunner.start();

        cameraThread.setState(Utilities.CAMERA_STATE.INIT);
        sleep(2000);
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
        sleep(50);
        cameraThread.setState(Utilities.CAMERA_STATE.KILL);

        Point firstSkystone = FieldStats.REDStones.getSkystone(2);
        Point secondSkystone = FieldStats.REDStones.getSkystone(1);

        telemetry.addData("First skystone", FieldStats.REDStones.getIndex(2));
        FieldStats.REDStones.displayPattern(telemetry);
        telemetry.update();

        positioningSystem.detach();
        sliderThreadPID.setTicks(1200);
        sleep(200);
        clawSystem.detach();

        if (FieldStats.REDStones.getIndex(2) == 4) {
            goToPositionNoTurning(firstSkystone, 1, globalPositionUpdate, robot, new PIDController(0.015, 0.0003, 0),5);
            clawSystem.lowerFlipper();
            sleep(600);
            sliderThreadPID.setTicks(600);

            goToPositionTurning(new Point(firstSkystone.x + 17.5, firstSkystone.y),1, Math.toRadians(-90),0.25, globalPositionUpdate, robot, telemetry, new PIDController(0.027,0.00087,0),35);
            positioningSystem.attach();
            // 0.6 0.8
            goToPositionTurning(new Point(firstSkystone.x + 17.5, 240), 1, Math.toRadians(0), 1, globalPositionUpdate, robot, telemetry, new PIDController(0.1, 0.01, 0), 3);
            clawSystem.attach();
            positioningSystem.detach();
            lifterThread.setTicks(LifterMethods.getTicksFromState(LifterMethods.LIFTER.FIRST));
            goToPositionTurning(new Point(firstSkystone.x, FieldStats.REDFoundation.DEFAULT_placingPosition.y), 1, Math.toRadians(-90), 0.8, globalPositionUpdate, robot, telemetry, new PIDController(0.1, 0.01, 0), 5);
            goToPositionNoTurning(new Point(firstSkystone.x - 15, FieldStats.REDFoundation.DEFAULT_placingPosition.y), 0.6, globalPositionUpdate, robot, new PIDController(0.12, 0.06, 0), 5);
            foundationSystem.attach();
            sleep(1000);
        } else {
            goToPositionNoTurning(firstSkystone, 1, globalPositionUpdate, robot, new PIDController(0.0092, 0.0005, 0),5);
            clawSystem.lowerFlipper();
            sleep(600);
            sliderThreadPID.setTicks(600);

            goToPositionTurning(new Point(firstSkystone.x + 17.5, firstSkystone.y),1, Math.toRadians(-90),0.25, globalPositionUpdate, robot, telemetry, new PIDController(0.027,0.00087,0),35);
            positioningSystem.attach();
            // 0.6 0.8
            goToPositionTurning(new Point(firstSkystone.x + 17.5, 240), 1, Math.toRadians(0), 1, globalPositionUpdate, robot, telemetry, new PIDController(0.1, 0.01, 0), 3);
            clawSystem.attach();
            positioningSystem.detach();
            lifterThread.setTicks(LifterMethods.getTicksFromState(LifterMethods.LIFTER.FIRST));
            goToPositionTurning(new Point(firstSkystone.x, FieldStats.REDFoundation.DEFAULT_placingPosition.y), 1, Math.toRadians(-90), 0.8, globalPositionUpdate, robot, telemetry, new PIDController(0.1, 0.01, 0), 5);
            goToPositionNoTurning(new Point(firstSkystone.x - 15, FieldStats.REDFoundation.DEFAULT_placingPosition.y), 0.6, globalPositionUpdate, robot, new PIDController(0.1, 0.01, 0), 5);
            foundationSystem.attach();
            sleep(1000);
        }


        //goToPositionTurning(new Point(firstSkystone.x + 17.5, 306), 1, Math.toRadians(-90), 0.4, globalPositionUpdate, robot, telemetry, new PIDController( 0.05, 0.008, 0), 3);

//        clawSystem.attach();
//        while (Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition) < FieldStats.REDFoundation.DEFAULT_placingPosition.y - 5 && opModeIsActive()) {
//            robot.frontLeftWheel.setPower(1);
//            robot.backLeftWheel.setPower(-1);
//            robot.frontRightWheel.setPower(-1);
//            robot.backRightWheel.setPower(1);
//        }
//        double reffY = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
//        double reffX = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
//        robot.frontLeftWheel.setPower(0);
//        robot.backLeftWheel.setPower(0);
//        robot.frontRightWheel.setPower(0);
//        robot.backRightWheel.setPower(0);
//        //goToPositionNoPID(new Point(firstSkystone.x + 17.5, FieldStats.REDFoundation.DEFAULT_placingPosition.y), 1, globalPositionUpdate, robot, new PIDController(0,0,0), 32);
//        positioningSystem.detach();
//        lifterThread.setTicks(LifterMethods.getTicksFromState(LifterMethods.LIFTER.FIRST));
//        goToPositionTurning(new Point(reffX - 20, reffY), 1, Math.toRadians(-90), 0.3, globalPositionUpdate, robot, telemetry, new PIDController(0.3, 0.008, 0), 5);
//        foundationSystem.attach();
//        sleep(780);
//        sliderThreadPID.setTicks(1500);
//        clawSystem.detach();
//
//        while (getZAngle() < 86) {
//            robot.frontLeftWheel.setPower(0);
//            robot.backLeftWheel.setPower(0);
//            robot.frontRightWheel.setPower(-1);
//            robot.backRightWheel.setPower(-1);
//        }
//        robot.frontLeftWheel.setPower(0);
//        robot.backLeftWheel.setPower(0);
//        robot.frontRightWheel.setPower(0);
//        robot.backRightWheel.setPower(0);
//
//        foundationSystem.detach();

        while (opModeIsActive()) {
            telemetry.addData("Distance left: ", FieldStats.REDFoundation.DEFAULT_placingPosition.y - Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition));
            telemetry.addData("Distance x: ", firstSkystone.x + 17.5 - Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition));
            telemetry.update();
        }
    }

    private double getZAngle() {
        return (-robot.imu.getAngularOrientation().firstAngle);
    }

    private void goToPositionNoTurning(Point target, double moveSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot, PIDController pidController, int tolerance) {
        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = Math.hypot(target.x - worldXPosition, target.y - worldYPosition);

        pidController.setSetpoint(initialDistance);
        pidController.setInputRange(0, initialDistance);
        pidController.setOutputRange(0, moveSpeed);
        pidController.setTolerance(tolerance);
        pidController.enable();

        do {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            worldAngle_rad = convertToUnitCircle(Math.toRadians(getZAngle() + initialOrientationDegrees));

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

            if( initialDistance - distanceToTarget < 5 && initialDistance > 15) powerFraction = 0.8;


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
            telemetry.addData("Error", pidController.getError());
            telemetry.addData("On target:", pidController.onTarget());
            telemetry.addData("Constants: ", pidController.getP() + " " + pidController.getI());
            telemetry.addData("Set Point", pidController.getSetpoint());
            telemetry.update();
        } while (opModeIsActive() && !pidController.onTarget());

        telemetry.addData("Finished", "|||");
        telemetry.update();

        robot.frontLeftWheel.setPower(0);
        robot.frontRightWheel.setPower(0);
        robot.backLeftWheel.setPower(0);
        robot.backRightWheel.setPower(0);

    }

    private void goToPositionNoPID(Point target, double moveSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot) {
        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = Math.hypot(target.x - worldXPosition, target.y - worldYPosition);
        double distanceToTarget;

        do {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            worldAngle_rad = convertToUnitCircle(Math.toRadians(getZAngle() + initialOrientationDegrees));

            distanceToTarget = Math.hypot(target.x - worldXPosition, target.y - worldYPosition);

            double absoluteAngleToTarget = Math.atan2(target.y - worldYPosition, target.x - worldXPosition);
            double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            movementXPower = Range.clip(movementXPower, -1, 1);
            movementYPower = Range.clip(movementYPower, -1, 1);

            double _movement_x = movementXPower;
            double _movement_y = movementYPower;


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
            telemetry.update();
        } while (opModeIsActive() && distanceToTarget > 2);

        telemetry.addData("Finished", "|||");
        telemetry.update();

        robot.frontLeftWheel.setPower(0);
        robot.frontRightWheel.setPower(0);
        robot.backLeftWheel.setPower(0);
        robot.backRightWheel.setPower(0);

    }

    public void goToPositionTurning(Point position, double movementSpeed, double preferredAngle, double turnSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot, Telemetry telemetry, PIDController pidController, int tolerance) {

        ElapsedTime timer = new ElapsedTime();

        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = Math.hypot(position.x - worldXPosition, position.y - worldYPosition);

        if (preferredAngle <= TWO_PI) preferredAngle += TWO_PI;
        if (preferredAngle >= TWO_PI) preferredAngle -= TWO_PI;
        preferredAngle = convertToUnitCircle(preferredAngle);

        pidController.setSetpoint(initialDistance);
        pidController.setInputRange(0, initialDistance);
        pidController.setOutputRange(0, movementSpeed);
        pidController.setTolerance(5);
        pidController.enable();

        do {

            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            worldAngle_rad = convertToUnitCircle(Math.toRadians(getZAngle() + initialOrientationDegrees));

            double distanceToTarget = Math.hypot(position.x - worldXPosition, position.y - worldYPosition);

            double absoluteAngleToTarget = Math.atan2(position.y - worldYPosition, position.x - worldXPosition);
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


            double _movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
            double _movement_x = movementXPower * powerFraction;
            double _movement_y = movementYPower * powerFraction;


            double drive = _movement_y;
            double strafe = -_movement_x;
            double turn = -_movement_turn;

            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

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

    public void goToPositionTurningnoPID(Point position, double movementSpeed, double preferredAngle, double turnSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot, Telemetry telemetry) {

        ElapsedTime timer = new ElapsedTime();

        double worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
        double worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
        double worldAngle_rad;

        double initialDistance = Math.hypot(position.x - worldXPosition, position.y - worldYPosition);
        double distanceToTarget;

        if (preferredAngle <= TWO_PI) preferredAngle += TWO_PI;
        if (preferredAngle >= TWO_PI) preferredAngle -= TWO_PI;
        preferredAngle = convertToUnitCircle(preferredAngle);

        do {

            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            worldAngle_rad = convertToUnitCircle(Math.toRadians(getZAngle() + initialOrientationDegrees));

            distanceToTarget = Math.hypot(position.x - worldXPosition, position.y - worldYPosition);

            double absoluteAngleToTarget = Math.atan2(position.y - worldYPosition, position.x - worldXPosition);
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
            double _movement_x = movementXPower;
            double _movement_y = movementYPower;

            double drive = _movement_y;
            double strafe = -_movement_x;
            double turn = -_movement_turn;

            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

            robot.frontLeftWheel.setPower(leftFrontPower);
            robot.frontRightWheel.setPower(rightFrontPower);
            robot.backLeftWheel.setPower(leftBackPower);
            robot.backRightWheel.setPower(rightBackPower);

            telemetry.addData(" Dist to target", distanceToTarget);
            telemetry.addData("Angle:", Math.toDegrees(worldAngle_rad));
            telemetry.addData("X pos:", worldXPosition);
            telemetry.addData("Y pos: ", worldYPosition);
            telemetry.update();

        } while (opModeIsActive() && distanceToTarget > 3);

        robot.frontLeftWheel.setPower(0);
        robot.frontRightWheel.setPower(0);
        robot.backLeftWheel.setPower(0);
        robot.backRightWheel.setPower(0);
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

//    private double getPowerByDistance(double distance) {
//        return Math.pow(distance, 3) * 1.593183 + Math.pow(distance, 2) * (-4.229217) + distance * 3.635241 - 0.001638;
//    }
//    private void resetAngle() {
//        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        globalAngle = 0;
//    }
//
//    private double getAngle() {
//        // We experimentally determined the Z axis is the axis we want to use for heading angle.
//        // We have to process the angle because the imu works in euler angles so the Z axis is
//        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
//
//        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;
//
//        lastAngles = angles;
//
//        return globalAngle;
//    }
//
//    private void rotate(int degrees, double power, PIDController pidRotate, Hardware robot) {
//        // restart imu angle tracking.
//        resetAngle();
//
//        // if degrees > 359 we cap at 359 with same sign as original degrees.
//        if (abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);
//
//        // start pid controller. PID controller will monitor the turn angle with respect to the
//        // target angle and reduce power as we approach the target angle. This is to prevent the
//        // robots momentum from overshooting the turn after we turn off the power. The PID controller
//        // reports onTarget() = true when the difference between turn angle and target angle is within
//        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
//        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
//        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
//        // turning the robot back toward the setpoint value.
//
//        pidRotate.reset();
//        pidRotate.setSetpoint(degrees);
//        pidRotate.setInputRange(0, degrees);
//        pidRotate.setOutputRange(0, power);
//        pidRotate.setTolerance(1);
//        pidRotate.enable();
//
//        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//        // clockwise (right).
//
//        // rotate until turn is completed.
//
//        if (degrees < 0) {
//            // On right turn we have to get off zero first.
//            while (opModeIsActive() && getAngle() == 0) {
//                robot.frontLeftWheel.setPower(power);
//                robot.backLeftWheel.setPower(power);
//                robot.frontRightWheel.setPower(-power);
//                robot.backRightWheel.setPower(-power);
//                sleep(100);
//            }
//
//            do {
//                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
//                robot.frontLeftWheel.setPower(-power);
//                robot.backLeftWheel.setPower(-power);
//                robot.frontRightWheel.setPower(power);
//                robot.backRightWheel.setPower(power);
//            } while (opModeIsActive() && !pidRotate.onTarget());
//        } else {    // left turn.
//            do {
//                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
//                robot.frontLeftWheel.setPower(-power);
//                robot.backLeftWheel.setPower(-power);
//                robot.frontRightWheel.setPower(power);
//                robot.backRightWheel.setPower(power);
//            } while (opModeIsActive() && !pidRotate.onTarget());
//        }
//
//        // turn the motors off.
//        robot.frontLeftWheel.setPower(0);
//        robot.backLeftWheel.setPower(0);
//        robot.frontRightWheel.setPower(0);
//        robot.backRightWheel.setPower(0);
//
//        rotation = getAngle();
//
//        // wait for rotation to stop.
//        sleep(500);
//
//        // reset angle tracking on new heading.
//        resetAngle();
//    }
}
