package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Utilities;
import org.firstinspires.ftc.teamcode.Misc.LifterMethods;
import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.text.DecimalFormat;
import java.text.NumberFormat;

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

    private LifterThread lifterThread = null;
    public LifterMethods.LIFTER currentState;
    private int levelIndex;

    private boolean leftBumperInput;
    private boolean rightBumperInput;

    private boolean buttonIsPressed = false;

    @Override
    public void init() {
        super.init();

        telemetry.setAutoClear(false);
        odometryTelemetry = telemetry.addData("Odometry - ", "");
        powerCoeffTelemetry = telemetry.addData("Power Coeff: ", 1);
        armTelemetry = telemetry.addData("Arm - ", "Lifter:  " + 0 + "  Extension:  " + 0);
        telemetry.update();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, Utilities.TICKS_PER_INCH, 75);

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        lifterThread = new LifterThread(robot.leftLifter, robot.rightLifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();
        currentState = LifterMethods.LIFTER.FLOAT;
        levelIndex = 0;
    }


    @Override
    public void loop() {
        super.loop();

        if (controllerInputA.leftBumperOnce() && powerCoeff > 0.2) {
            powerCoeff -= 0.1;
        } else if (controllerInputA.rightBumperOnce() && powerCoeff <= 1) {
            powerCoeff += 0.1;
        }
        powerCoeff = Range.clip(powerCoeff, 0.0, 1.0);

        //------------------DRIVING-----------------------
        float drive, turn, strafe;

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
        if (robot.button.isPressed()) {  //end all motion completely
            buttonIsPressed = true;
            robot.leftLifter.setPower(0);
            robot.rightLifter.setPower(0);
            robot.leftLifter.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftLifter.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        }
        else buttonIsPressed = false;

        lifterNewPower = gamepad2.right_stick_y;
        if (lifterNewPower != lifterOldPower && LifterThread.finished) {

            if(buttonIsPressed && lifterNewPower > 0) lifterNewPower = 0;
            if (lifterNewPower < -0.8) lifterNewPower = lifterNewPower * 0.7;
            robot.leftLifter.setPower(-lifterNewPower);
            robot.rightLifter.setPower(-lifterNewPower);
        }
        lifterOldPower = lifterNewPower;

        if (controllerInputB.dpadLeftOnce() && levelIndex > 0) levelIndex--;
        if (controllerInputB.dpadRightOnce() && levelIndex < 6) levelIndex++;

        if (controllerInputB.dpadUpOnce() && LifterThread.finished) {

            lifterThread.setTicks(LifterMethods.getTicksFromState(level[levelIndex]));
            currentState = level[levelIndex];
        }
        currentState = LifterMethods.getStateFromTicks(robot.leftLifter.getCurrentPosition());


        //-------------------SERVO CONTROL-----------------------------


        updateTelemetry();
    }

    @Override
    public void stop() {
        super.stop();
    }

    private void updateTelemetry() {
        NumberFormat formatter = new DecimalFormat("#0.000");
        odometryTelemetry.setValue("X: " + formatter.format(Utilities.TICKS_TO_CM(globalPositionUpdate.returnXCoordinate()))
                + "Y: " + formatter.format(Utilities.TICKS_TO_CM(globalPositionUpdate.returnYCoordinate())) + "  Angle: "
                + formatter.format(globalPositionUpdate.returnOrientationDeg()) + "  Alive: "
                + globalPositionUpdate.isRunning);
        powerCoeffTelemetry.setValue(powerCoeff);
        armTelemetry.setValue("Lifter:  " + robot.leftLifter.getCurrentPosition() + "  Next level:  " + levelIndex
                + " Thread: " + LifterThread.finished + "  Button: " + robot.button.isPressed());
        telemetry.update();
    }

}