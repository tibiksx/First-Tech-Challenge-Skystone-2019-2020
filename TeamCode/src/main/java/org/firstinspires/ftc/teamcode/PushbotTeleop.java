
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.ControllerInput;
import org.firstinspires.ftc.teamcode.utils.LifterThread;

@TeleOp(name="Pushbot: Teleop", group="Pushbot")

public class PushbotTeleop extends OpMode {

    Hardware robot = new Hardware();

    double posFound1 = 0.0, posFound2 = 0.0;
    double coeff = 1.0;
    int i = 0;

    public ControllerInput controllerInputA;
    public ControllerInput controllerInputB;

    public LifterThread lifterThread;


    public enum LIFTER {
        LOW,
        FIRST,
        SECOND,
        THIRD,
        FOURTH
    }

    public LIFTER currentState;
    LIFTER[] level = {LIFTER.LOW, LIFTER.FIRST, LIFTER.SECOND, LIFTER.THIRD, LIFTER.FOURTH};

    double oldPower = 0;
    double newPower = 0;

    public Telemetry.Item state;
    public Telemetry.Item threadState;
    public Telemetry.Item lifterTicks;
    public Telemetry.Item lifterLevel;

    public Telemetry.Item leftEncoder;
    public Telemetry.Item rightEncoder;
    public Telemetry.Item backEncoder;

    @Override
    public void init() {

        robot.init(hardwareMap);

        controllerInputA = new ControllerInput(gamepad1);
        controllerInputB = new ControllerInput(gamepad2);

        robot.frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.lifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.lifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        currentState = LIFTER.LOW;

        lifterThread = new LifterThread(robot.lifterLeft, robot.lifterRight);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();


        telemetry.setAutoClear(false);
        state = telemetry.addData("nivel:",currentState);
        lifterTicks = telemetry.addData("ticks uri:", robot.lifterLeft.getCurrentPosition());
        threadState = telemetry.addData("stare thread", LifterThread.finished);
        lifterLevel = telemetry.addData("level:", level[i]);

        leftEncoder = telemetry.addData("left encoder @ ", getEncoderPosition('l'));
        rightEncoder = telemetry.addData("right encoder @ ", getEncoderPosition('r'));
        backEncoder = telemetry.addData("back encoder @ ", getEncoderPosition('b'));
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    // Execute until driver hits STOP
    @Override
    public void loop() {

        if (controllerInputA.leftBumperOnce() && coeff > 0.2) {
            coeff -= 0.2;
        } else if (controllerInputA.rightBumperOnce() && coeff <= 1) {
            coeff += 0.2;
        }

        coeff = Range.clip(coeff, 0.0, 1.0);

        float drive, turn, strafe;

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        // clip() = demands a number to be in certain bounds
        // number is calculated and then processed
        double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

        robot.frontLeftWheel.setPower(coeff * leftFrontPower);
        robot.frontRightWheel.setPower(coeff * rightFrontPower);
        robot.backLeftWheel.setPower(coeff * leftBackPower);
        robot.backRightWheel.setPower(coeff * rightBackPower);

        /////// LIFTER ///////

        newPower = gamepad2.right_stick_y;

        if(newPower != oldPower) {
            robot.lifterLeft.setPower(newPower);
            robot.lifterRight.setPower(newPower);
        }
        oldPower = newPower;

        if (controllerInputB.dpadLeftOnce() && i > 0) {

            i--;

        }

        if (controllerInputB.dpadRightOnce() && i < 4) {

            i++;
        }

        if (controllerInputB.dpadUpOnce() && LifterThread.finished) {

            lifterThread.setTicks(getTicksFromState(level[i]));
        }

        if (controllerInputB.dpadDownOnce() && LifterThread.finished) {

            lifterThread.setTicks(getTicksFromState(level[i]));
        }

        lifterTicks.setValue(robot.lifterLeft.getCurrentPosition());
        lifterLevel.setValue(level[i]);

        leftEncoder.setValue(getEncoderPosition('l'));
        rightEncoder.setValue(getEncoderPosition('r'));
        backEncoder.setValue(getEncoderPosition('b'));

        telemetry.update();

        controllerInputA.update();
        controllerInputB.update();

    }

    @Override
    public void stop() {

        telemetry.log().clear();
        telemetry.addData("Currently in:", "STOP");
        telemetry.update();
    }

    int getTicksFromState(LIFTER currentState) {
        switch (currentState) {
            case LOW:
                return 0;
            case FIRST:
                return 1400;
            case SECOND:
                return 3000;
            case THIRD:
                return 4800;
            case FOURTH:
                return 6300;
        }
        return -1; //code should never reach here. Yes, this function sucks...
    }

    int getEncoderPosition(char encoderInitial) {

        if (encoderInitial == 'l') {
            return robot.verticalLeft.getCurrentPosition();
        } else if (encoderInitial == 'r') {
            return robot.verticalRight.getCurrentPosition();
        } else if (encoderInitial == 'b') {
            return robot.horizontal.getCurrentPosition();
        } else {
            return 666;
        }
    }
}


