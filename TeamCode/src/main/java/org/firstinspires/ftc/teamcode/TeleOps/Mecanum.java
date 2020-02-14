package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.*;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;
import org.firstinspires.ftc.teamcode.Threads.LifterThreadPID;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.Misc.LifterMethods.level;

@TeleOp(name = "Main TeleOp", group = "Pushbot")
public class Mecanum extends Robot {

    private Telemetry.Item odometryTelemetry = null;
    private Telemetry.Item powerCoeffTelemetry = null;
    private Telemetry.Item armTelemetry = null;

    private double lifterOldPower = 0;
    private double lifterNewPower = 0;

    private double powerCoeff = 1;

    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    private LifterThreadPID lifterThread = null;
    public LifterMethods.LIFTER currentState;
    private int levelIndex;

    private boolean leftBumperInput;
    private boolean rightBumperInput;

    private boolean newButtonState = false;
    private boolean oldButtonState = false;

    private final double initialXCoordinate = 0;
    private final double initialYCoordinate = 0;
    private final double initialOrientationDegrees = 0;

    private PositioningSystem positioningSystem;
    private ClawSystem clawSystem;
    private FoundationSystem foundationSystem;

    @Override
    public void init() {
        super.init();

        telemetry.setAutoClear(false);
        odometryTelemetry = telemetry.addData("Odometry - ", "");
        powerCoeffTelemetry = telemetry.addData("Power Coeff: ", 1);
        armTelemetry = telemetry.addData("Arm - ", "Lifter:  " + 0 + "  Extension:  " + 0);
        telemetry.update();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight
                , robot.horizontal, Utilities.TICKS_PER_INCH, 75);
        globalPositionUpdate.setInitialCoordinates(initialXCoordinate, initialYCoordinate, Math.toRadians(initialOrientationDegrees));
        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        lifterThread = new LifterThreadPID(robot.leftLifter, robot.rightLifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();
        currentState = LifterMethods.LIFTER.FLOAT;
        levelIndex = 0;

        clawSystem = new ClawSystem(robot.claw, robot.flipper);
        clawSystem.Initial();
        clawSystem.raiseFlipper();

        positioningSystem = new PositioningSystem(robot.posLeft, robot.posRight);
        positioningSystem.Initial();

        foundationSystem = new FoundationSystem(robot.foundationLeft,robot.foundationRight);
        foundationSystem.Detach();
    }


    @Override
    public void loop() {
        super.loop();

        if (controllerInputA.leftBumperOnce() && powerCoeff > 0.2) {
            powerCoeff -= 0.1;
        } else if (controllerInputA.rightBumperOnce() && powerCoeff <= 1) {
            powerCoeff += 0.1;
        }
        powerCoeff = Range.clip(powerCoeff, 0.1, 1.0);

        //------------------DRIVING-----------------------
        double drive, turn, strafe;

        drive = -gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

        robot.frontLeftWheel.setPower(powerCoeff * leftFrontPower);
        robot.frontRightWheel.setPower(powerCoeff * rightFrontPower);
        robot.backLeftWheel.setPower(powerCoeff * leftBackPower);
        robot.backRightWheel.setPower(powerCoeff * rightBackPower);


        //-----------------SLIDE ARM----------------------------
        leftBumperInput = gamepad2.left_bumper;
        rightBumperInput = gamepad2.right_bumper;
        if (!leftBumperInput && !rightBumperInput)
            robot.slider.setPower(0);
        else if (leftBumperInput && !rightBumperInput)
            robot.slider.setPower(-1);
        else if (!leftBumperInput)
            robot.slider.setPower(1);


        //--------------------LIFTER------------------------------------

        newButtonState = robot.button.isPressed();
        if (newButtonState && !oldButtonState) {    //raising edge of the signal
            //kill the motors running in thread
            LifterThread.finished = true;
            robot.leftLifter.setPower(0);
            robot.rightLifter.setPower(0);
            robot.leftLifter.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftLifter.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
            currentState = LifterMethods.LIFTER.LOW;
        }
        oldButtonState = newButtonState;

        lifterNewPower = gamepad2.right_stick_y;
        if (lifterNewPower != lifterOldPower && LifterThread.finished) {

            if (newButtonState && lifterNewPower > 0) { //if button is pressed limit movement to upwards only
                lifterNewPower = 0;
            }
            robot.leftLifter.setPower(-lifterNewPower);
            robot.rightLifter.setPower(-lifterNewPower);
        }
        lifterOldPower = lifterNewPower;

        if (controllerInputB.dpadLeftOnce() && levelIndex > 0) levelIndex--;
        if (controllerInputB.dpadRightOnce() && levelIndex < 8) levelIndex++;

        if ((controllerInputB.dpadUpOnce() || controllerInputB.dpadDownOnce()) && LifterThread.finished) {
            lifterThread.setTicks(LifterMethods.getTicksFromState(level[levelIndex]));
            currentState = level[levelIndex];
        }
        currentState = LifterMethods.getStateFromTicks(robot.leftLifter.getCurrentPosition());

        //-------------------SERVO CONTROL-----------------------------

        if (controllerInputA.BOnce()) { //  deal with the claw
            if (!clawSystem.isAttached())
            {
                clawSystem.Attach(); //when attaching claw, wait a little then detach the positioning system
                sleep(500);
                positioningSystem.Detach();
                sleep(200);
            }
            else clawSystem.Detach();
        }

        if (controllerInputB.AOnce()) // deal with the positioning system
        {
            if (positioningSystem.isInitial())
                positioningSystem.Detach();
            else if (positioningSystem.isAttached())
                positioningSystem.Detach();
            else positioningSystem.Attach();
        }

        if (controllerInputB.YOnce()) { //deal with the flipper
            if (clawSystem.isLowered())
                clawSystem.raiseFlipper();
            else clawSystem.lowerFlipper();
        }

        if(controllerInputA.AOnce()) { //deal with the foundation system
            if(foundationSystem.isAttached())
                foundationSystem.Detach();
            else foundationSystem.Attach();
        }

//
        updateTelemetry();

    }

    private void updateTelemetry() {
        NumberFormat formatter = new DecimalFormat("#0.000");
        odometryTelemetry.setValue("X: " + formatter.format(Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalXCoordinatePosition))
                + "  Y: " + formatter.format(Utilities.TICKS_TO_CM(globalPositionUpdate.robotGlobalYCoordinatePosition)) + "  Angle: "
                + formatter.format(globalPositionUpdate.robotOrientationDeg) + "  Alive: "
                + globalPositionUpdate.isRunning);
        powerCoeffTelemetry.setValue(powerCoeff);
        armTelemetry.setValue("Lifter:  " + robot.leftLifter.getCurrentPosition() + "  Next level:  " + levelIndex
                + "  Button: " + robot.button.isPressed() + " Slider  " + robot.slider.getCurrentPosition());
        telemetry.update();
    }


    private void moveSlider(int position) {
        robot.slider.setTargetPosition(position);
        robot.slider.setTargetPositionTolerance(50);
        robot.slider.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
        if (robot.slider.getCurrentPosition() < position) {
            robot.slider.setPower(1);
            while (robot.slider.getCurrentPosition() < position - 100) {
                telemetry.addData("POZ", robot.slider.getPower() + " " + robot.slider.getCurrentPosition());
                telemetry.update();
            }
        } else {
            robot.slider.setPower(-1);
            while (robot.slider.getCurrentPosition() > position + 100) {
                telemetry.addData("POZ", robot.slider.getPower() + " " + robot.slider.getCurrentPosition());
                telemetry.update();
            }
        }
        robot.slider.setPower(0);
        sleep(50);
    }

}