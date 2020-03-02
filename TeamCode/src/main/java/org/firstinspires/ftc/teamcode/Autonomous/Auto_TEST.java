package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Misc.ClawSystem;
import org.firstinspires.ftc.teamcode.Misc.FoundationSystem;
import org.firstinspires.ftc.teamcode.Misc.LifterMethods;
import org.firstinspires.ftc.teamcode.Misc.PIDController;
import org.firstinspires.ftc.teamcode.Misc.PositioningSystem;
import org.firstinspires.ftc.teamcode.Misc.Utilities;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;

@Autonomous(name = "Auto Test", group = "Test")
public class Auto_TEST extends LinearOpMode {

    private Hardware robot = new Hardware();
    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    private LifterThread lifterThread = null;
    public LifterMethods.LIFTER currentState;
    private int levelIndex;

    private ClawSystem clawSystem = null;
    private PositioningSystem positioningSystem = null;
    private FoundationSystem foundationSystem = null;


    private final double initialXCoordinate = 0;
    private final double initialYCoordinate = 0;
    private final double initialOrientationDegrees = 0;

    Orientation lastAngles = new Orientation();
    double globalAngle;
    double rotation;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.initIMU();
        robot.initWebcam();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal,robot.ExpansionHub1, Utilities.TICKS_PER_INCH, 75);
        globalPositionUpdate.setInitialCoordinates(initialXCoordinate, initialYCoordinate, Math.toRadians(initialOrientationDegrees));
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

        telemetry.addData("Calibrated","GO");
        waitForStart();

        //TODO test this
        //Utilities.goToPositionNoTurning(-60,-60,0.6,globalPositionUpdate,robot,telemetry);

        //TODO test this too
//        PIDController pidRotate = new PIDController(0.018, 0.0002, 0.00001);
//        rotate(-45,1,pidRotate,robot);
//        sleep(500);
//        rotate(-90,0.8,pidRotate,robot);

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
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

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
}
