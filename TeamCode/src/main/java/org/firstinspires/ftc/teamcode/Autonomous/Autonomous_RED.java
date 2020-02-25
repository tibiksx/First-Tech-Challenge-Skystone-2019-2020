package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ComputerVision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.FieldStats;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Misc.*;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;
import org.firstinspires.ftc.teamcode.Threads.SliderThreadPID;
import org.opencv.core.Point;
import org.openftc.revextensions2.ExpansionHubMotor;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Misc.Utilities.AngleWrap;

@Autonomous(name = "Auto RED", group = "Autonomous")
public class Autonomous_RED extends LinearOpMode {

    private Hardware robot = new Hardware();
    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    private LifterThread lifterThread = null;
    public LifterMethods.LIFTER currentState;
    private int levelIndex;

    private ClawSystem clawSystem = null;
    private PositioningSystem positioningSystem = null;
    private FoundationSystem foundationSystem = null;

    private SkystoneDetector skystoneDetector;

    private final double initialXCoordinate =0;//= Utilities.CM_TO_TICKS(336);
    private final double initialYCoordinate =0;//= Utilities.CM_TO_TICKS(98);
    private final double initialOrientationDegrees = 0;//-90;

    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double rotation;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initIMU();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, Utilities.TICKS_PER_INCH, 50);
        globalPositionUpdate.setInitialCoordinates(initialXCoordinate, initialYCoordinate, toRadians(initialOrientationDegrees));
        globalPositionUpdate.reverseRightEncoder();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        clawSystem = new ClawSystem(robot.claw, robot.flipper);
        clawSystem.Initial();
        clawSystem.raiseFlipper();

        positioningSystem = new PositioningSystem(robot.posLeft, robot.posRight);
        positioningSystem.Initial();

        foundationSystem = new FoundationSystem(robot.foundationLeft,robot.foundationRight);
        foundationSystem.Detach();


        lifterThread = new LifterThread(robot.leftLifter, robot.rightLifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();
        currentState = LifterMethods.LIFTER.FLOAT;
        levelIndex = 0;

        SliderThreadPID sliderThreadPID = new SliderThreadPID(robot.slider);
        Thread sliderThread = new Thread(sliderThreadPID);
        sliderThread.start();

        skystoneDetector = new SkystoneDetector(hardwareMap,telemetry);
        sleep(50);
        //skystoneDetector.init();

        PIDController pidRotate = new PIDController(0.025, 0.0001, 0.00001);

        waitForStart();

        //goToPositionNoTurning(FieldStats.REDFoundation.DEFAULT_placingPosition.x-30 ,FieldStats.REDFoundation.DEFAULT_placingPosition.y,0.7,globalPositionUpdate,robot);
        goToPositionTurning(60,60,0.7,Math.toRadians(90),0.8,globalPositionUpdate,robot,telemetry);



//        int[] stoneValues = skystoneDetector.scan();
//        int valLeft = stoneValues[0];
//        int valMid = stoneValues[1];
//        int valRight = stoneValues[2];
//        skystoneDetector.killCamera();
//
//        if (valLeft == 0 && valMid == 255 && valRight == 255) {  //left stone is a skystone
//            FieldStats.REDStones.markAsSkystone(4);
//            FieldStats.REDStones.markAsSkystone(4 - 3);
//        } else if (valLeft == 255 && valMid == 0 && valRight == 255) { //mid stone is a skystone
//            FieldStats.REDStones.markAsSkystone(5);
//            FieldStats.REDStones.markAsSkystone(5 - 3);
//        } else if (valLeft == 255 && valMid == 255 && valRight == 0) {  //right stone is a skystone
//            FieldStats.REDStones.markAsSkystone(6);
//            FieldStats.REDStones.markAsSkystone(6 - 3);
//        }
//        telemetry.log().clear();
//        FieldStats.REDStones.displayPattern(telemetry);
//
//        //find the closest stone to pick up
//        FieldStats.Stone closestStone1 = new FieldStats.Stone(new Point(-1,-1),false);
//        int closestStoneIndex = -1;
//        for(int i = 6; i>=1; i--) {
//            if(FieldStats.REDStones.stoneArray[i].isSkystone()) {
//                closestStone1 = FieldStats.REDStones.stoneArray[i];
//                closestStoneIndex = i;
//                break;
//            }
//        }
//        telemetry.addData("Robot",Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition) + "  " + Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition) + " " + globalPositionUpdate.robotOrientationDeg);
//        telemetry.addData("Skystone: ",closestStone1.getPosition().x + " " + closestStone1.getPosition().y);
//        telemetry.update();
//        sleep(2000);
//        //only if we found a skystone
//        if(!(closestStone1.getPosition().x == -1 && closestStone1.getPosition().y == -1)) {
//
//            telemetry.addData("Moving to: ",closestStone1.getPosition().x + "  " + closestStone1.getPosition().y);
//            telemetry.update();
//            //move slider forward
//            sliderThreadPID.setTicks(1200);
//            //go to the stone's position
//            Utilities.goToPositionNoTurning(closestStone1.getPosition().x, closestStone1.getPosition().y, 0.7, globalPositionUpdate, robot,telemetry);
//            sleep(500);
//            //catch it
//            clawSystem.Attach();
//            sleep(600);
//            //pull it back
//            sliderThreadPID.setTicks(670);
//
//            telemetry.addData("Caught the first skystone","");
//            telemetry.update();
//            sleep(200);
//            //back off a little
//            Utilities.goToPositionNoTurning(closestStone1.getPosition().x-20, closestStone1.getPosition().y, 0.7, globalPositionUpdate, robot,telemetry);
//            sleep(1000);
//            Utilities.goToPositionNoTurning(FieldStats.REDFoundation.DEFAULT_placingPosition.x,FieldStats.REDFoundation.DEFAULT_placingPosition.y,0.5,globalPositionUpdate,robot,telemetry);
//            sleep(1000);
//            foundationSystem.Attach();
//            sleep(500);
//            //TODO Arc here
//            foundationSystem.Detach();
//
//            //find the next skystone
//            FieldStats.Stone closestStone2 = new FieldStats.Stone(new Point(-1,-1),false);
//            for(int i = closestStoneIndex-1; i>=1; i--) {
//                if(FieldStats.REDStones.stoneArray[i].isSkystone()) {
//                    closestStone2 = FieldStats.REDStones.stoneArray[i];
//                    closestStoneIndex = i;
//                    break;
//                }
//            }
//
//
//        }

    }

    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
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

    private void rotate(int degrees, double power, PIDController pidRotate, Hardware robot)
    {
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

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                robot.frontLeftWheel.setPower(power);
                robot.backLeftWheel.setPower(power);
                robot.frontRightWheel.setPower(-power);
                robot.backRightWheel.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.frontLeftWheel.setPower(-power);
                robot.backLeftWheel.setPower(-power);
                robot.frontRightWheel.setPower(power);
                robot.backRightWheel.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
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

    private double iccKinematics(double radius) {
        return (radius - 20) / (radius + 20);
    }

    private double getZAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle);
    }


    public void goToPositionNoTurning(double xGoal, double yGoal,double moveSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot) {
        double worldXPosition;
        double worldYPosition;
        double worldAngle_rad;


        while(opModeIsActive()) {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            worldAngle_rad = covertToUnitCircle(globalPositionUpdate.robotOrientationRadians);

            double distToTarget = Math.hypot(xGoal - worldXPosition, yGoal - worldYPosition);
            double absoluteAngleToTarget = Math.atan2(yGoal - worldYPosition, xGoal - worldXPosition);
            double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distToTarget;

            double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));

            movementXPower = Range.clip(movementXPower,-1,1);
            movementYPower = Range.clip(movementYPower, -1 , 1);

            double _movement_x = movementXPower * moveSpeed;
            double _movement_y = movementYPower * moveSpeed;

            if(abs(relativeXToPoint) < 4) _movement_x = 0;
            if(abs(relativeYToPoint) < 4) _movement_y = 0;

            if(distToTarget < 15)  {
                _movement_x = _movement_x * 0.7;
                _movement_y = _movement_y * 0.7;
            }

            if(distToTarget < 5) {
                robot.frontLeftWheel.setPower(0);
                robot.frontRightWheel.setPower(0);
                robot.backLeftWheel.setPower(0);
                robot.backRightWheel.setPower(0);
                break;
            }

            double d = _movement_y;
            double t = 0;
            double s = -_movement_x;
            double fl = d + t - s;
            double bl = d + t + s;
            double fr = d - t + s;
            double br = d - t - s;

            robot.frontLeftWheel.setPower(fl);
            robot.frontRightWheel.setPower(fr);
            robot.backLeftWheel.setPower(bl);
            robot.backRightWheel.setPower(br);
        }

    }



    public void goToPositionTurning(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, OdometryGlobalCoordinatePosition globalPositionUpdate, Hardware robot, Telemetry telemetry) {

        double worldXPosition;
        double worldYPosition;
        double worldAngle_rad;

        preferredAngle = covertToUnitCircle(preferredAngle);

        while(opModeIsActive()) {
            worldXPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition);
            worldYPosition = Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition);
            worldAngle_rad = covertToUnitCircle(Math.toRadians(getZAngle()));

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

            if(preferredAngle >= TWO_PI) preferredAngle-= TWO_PI;
            double relativeTurnAngle = preferredAngle - worldAngle_rad;

            relativeTurnAngle = AngleWrap(relativeTurnAngle);

            double _movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
            double _movement_x = movementXPower * movementSpeed;
            double _movement_y = movementYPower * movementSpeed;


            if(distanceToTarget < 15){
                _movement_x = _movement_x * 0.7;
                _movement_y = _movement_y * 0.7;
            } else if(distanceToTarget < 7) {
                _movement_x = _movement_x * 0.5;
                _movement_y = _movement_y * 0.5;
            }


            double drive = _movement_y;
            double strafe = -_movement_x;
            double turn = -_movement_turn;

            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

            if(  (distanceToTarget < 5) || (_movement_x == 0 && _movement_y == 0 && _movement_turn == 0) ) break;

            robot.frontLeftWheel.setPower(leftFrontPower);
            robot.frontRightWheel.setPower(rightFrontPower);
            robot.backLeftWheel.setPower(leftBackPower);
            robot.backRightWheel.setPower(rightBackPower);

            telemetry.addData(" Dist to target",distanceToTarget);
            telemetry.addData("Angle:",globalPositionUpdate.robotOrientationDeg);
            telemetry.update();


        }

    }



    final static double TWO_PI = 2 * PI;
    final static double HALF_PI = PI / 2;

    private double covertToUnitCircle(double angle) {
        //reduce back to unit circle
        if(angle > TWO_PI) angle -= TWO_PI;
        if(angle < 0) angle += TWO_PI;
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

}
