package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {

    Hardware robot = new Hardware();

    final double DEFAULT_PID = .02;

    final double COUNTS_PER_REV = 1600;
    final double PI = Math.PI;
    final double WHEEL_DIAMETER_INCHES = 1.96;
    ///////////////////////////////////////////
    final double COUNTS_PER_INCH = COUNTS_PER_REV / (PI * WHEEL_DIAMETER_INCHES);

    final double CENTER_WHEELS = 13.5 / 2;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();


        //moveInArcTo(25*COUNTS_PER_INCH,25*COUNTS_PER_INCH, 0.6, DEFAULT_PID);
//        sleep(2000);
//        goToPosition(20 * COUNTS_PER_INCH, 20 * COUNTS_PER_INCH, 0.5, 60, 2 * COUNTS_PER_INCH, DEFAULT_PID);
//        sleep(2000);
//        goToPosition(30 * COUNTS_PER_INCH, 30 * COUNTS_PER_INCH, 0.5, 90, 2 * COUNTS_PER_INCH, DEFAULT_PID);
        goToPositionFinal(20*COUNTS_PER_INCH, 20*COUNTS_PER_INCH, .4, 90, 5 * COUNTS_PER_INCH);
        while(opModeIsActive()){

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        globalPositionUpdate.stop();

    }

    public void moveInArcTo(double targetX, double targetY, double power) {

        double radius = Math.abs(targetX - (globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH));

        double radiusRight = radius - CENTER_WHEELS;
        double radiusLeft = radius + CENTER_WHEELS;

        double coeff = radiusLeft/radiusRight;

        telemetry.addData("radius", radius);
        telemetry.addData("radiusleft", radiusLeft);
        telemetry.addData("radiusright", radiusRight);
        telemetry.addData("coeff", coeff);
        telemetry.update();

        while (opModeIsActive() && -robot.imu.getAngularOrientation().firstAngle < 90) {

            setPowerAll(power, power, power * coeff, power * coeff);
        }

        setPowerAll(0, 0, 0, 0);
    }

    public void goToPositionFinal(double targetX, double targetY, double power, double desiredOrientation, double error) {

        double distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
        double distanceToY = targetY - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToX, distanceToY);

        while (opModeIsActive() && distance > error) {

            distance = Math.hypot(distanceToX, distanceToY);

            distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToY = targetY - globalPositionUpdate.returnYCoordinate();

            if (distanceToX < 0) {
                distanceToX = 0;
            }

            if (distanceToY < 0) {
                distanceToY = 0;
            }

            //telemetry.log().clear();
            telemetry.addData(" Distance to x: ",distanceToX);
            telemetry.addData(" Distance to y: ",distanceToY);
            telemetry.addData("Distance: ",distance);
            telemetry.update();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            double xComponent = calculateX(robotMovementAngle, power);
            double yComponent = calculateY(robotMovementAngle, power);
            double phiComponent = (Math.abs(desiredOrientation) - Math.abs(robot.imu.getAngularOrientation().firstAngle)) * 0.02;

            if (distanceToX < 0) {
                distanceToX = 0;
                xComponent = 0;
            }

            if (distanceToY < 0) {
                distanceToY = 0;
                yComponent = 0;
            }

            if (Math.abs(robot.imu.getAngularOrientation().firstAngle) > 88) {
                phiComponent = 0.0;
            }
            telemetry.update();
            double leftFrontPower = Range.clip(yComponent + phiComponent - xComponent, 0, 1.0);
            double leftBackPower = Range.clip(yComponent + phiComponent + xComponent, 0, 1.0);
            double rightFrontPower = Range.clip(yComponent - phiComponent + xComponent, 0, 1.0);
            double rightBackPower = Range.clip(yComponent - phiComponent - xComponent, 0, 1.0);

            robot.frontLeftWheel.setPower(round2D(leftFrontPower));
            robot.frontRightWheel.setPower(round2D(rightFrontPower));
            robot.backLeftWheel.setPower(round2D(leftBackPower));
            robot.backRightWheel.setPower(round2D(rightBackPower));
        }

        setPowerAll(0.0, 0.0, 0.0, 0.0);
    }

    public void goToPosition(double targetXPosition, double targetYPosition, double power, double desiredOrientation, double dError, double pidGain) {
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while (opModeIsActive() && distance > dError) {

            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robotMovementXComp = calculateX(robotMovementAngle, power);
            double robotMovementYComp = calculateY(robotMovementAngle, power);
            double pivotCorrection = ((globalPositionUpdate.returnOrientation()-desiredOrientation) * pidGain);

            rawMovement(round2D(robotMovementXComp), round2D(robotMovementYComp), round2D(pivotCorrection), power);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.horizontal.getCurrentPosition());

            telemetry.update();

        }



        robot.frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerAll(0, 0, 0, 0);
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    public void rawMovement(double horizontal, double vertical, double pivot, double maxPower){

        double powers[] = {vertical - horizontal + pivot, vertical + horizontal + pivot, vertical + horizontal - pivot, vertical - horizontal - pivot};

        if (horizontal != 0 || vertical != 0) {

            int max = 0;
            int counter = 0;

            for (double element : powers) {

                if (Math.abs(element) > Math.abs(powers[max])) {
                    max = counter;
                }
                counter++;
            }

            double maxCalculatedPower = Math.abs(powers[max]);

            if(maxCalculatedPower != 0) {
                powers[0] = powers[0] / maxCalculatedPower * maxPower;
                powers[1] = powers[1] / maxCalculatedPower * maxPower;
                powers[2] = powers[2] / maxCalculatedPower * maxPower;
                powers[3] = powers[3] / maxCalculatedPower * maxPower;

            }
        }
        setPowerAll(powers[0], powers[1], powers[2], powers[3]);
    }

    public double round2D(double input) {
        input *= 100;
        input = Math.round(input);
        return input / 100;
    }

    private void setPowerAll(double rf, double rb, double lf, double lb){
        robot.frontRightWheel.setPower(rf);
        robot.backRightWheel.setPower(rb);
        robot.frontLeftWheel.setPower(lf);
        robot.backLeftWheel.setPower(lb);
    }

    public double[] goToPositionVlad(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        double distanceToTarget = Math.hypot(x - globalPositionUpdate.returnXCoordinate(), y - globalPositionUpdate.returnYCoordinate());

        if (distanceToTarget < 10) {
            return new double[]{0,0,0};

        }

        double absoluteAngleToTarget = Math.atan2(y - globalPositionUpdate.returnYCoordinate(), x - globalPositionUpdate.returnXCoordinate());
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (Math.toRadians(globalPositionUpdate.returnOrientation()) - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        double _movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
        double _movement_x = movementXPower * movementSpeed;
        double _movement_y = movementYPower * movementSpeed;

        return new double[]{_movement_x*-1,_movement_y,_movement_turn};

    }

    public double AngleWrap(double angle)
    {
        while(angle < - Math.PI)
            angle += 2 * Math.PI;

        while (angle > Math.PI)
            angle -= 2 * Math.PI;

        return angle;
    }
}