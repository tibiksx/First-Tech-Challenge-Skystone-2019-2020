package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.*;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;
import org.firstinspires.ftc.teamcode.Threads.SliderThreadPID;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import static org.firstinspires.ftc.teamcode.Misc.LifterMethods.level;

@TeleOp(name = "Main TeleOp", group = "Pushbot")
public class Mecanum extends Robot {

    private Telemetry.Item powerCoeffTelemetry = null;
    private Telemetry.Item armTelemetry = null;

    private double lifterOldPower = 0;
    private double lifterNewPower = 0;

    private double powerCoeff = 1;

    private LifterThread lifterThread = null;
    public LifterMethods.LIFTER currentState;
    private int levelIndex;

    private SliderThreadPID sliderThreadPID;
    private int[] sliderStates = {200,1000};
    private int sliderIndex = -1;

    private boolean oldLeftBumper;
    private boolean oldRightBumper;
    private boolean newLeftBumper;
    private boolean newRightBumper;

    private int oldSliderPower;
    private int newSliderPower;

    private boolean newButtonState = false;
    private boolean oldButtonState = false;

    private PositioningSystem positioningSystem;
    private ClawSystem clawSystem;
    private FoundationSystem foundationSystem;

    private boolean startButton1 = false;
    private boolean startButton2 = false;

    @Override
    public void init() {
        super.init();

        robot.ExpansionHub2.setPhoneChargeEnabled(true);

        telemetry.setAutoClear(false);
        powerCoeffTelemetry = telemetry.addData("Power Coeff: ", 1);
        armTelemetry = telemetry.addData("Arm - ", "Lifter:  " + 0 + "  Extension:  " + 0);
        telemetry.update();


        lifterThread = new LifterThread(robot.leftLifter, robot.rightLifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();
        currentState = LifterMethods.LIFTER.FLOAT;
        levelIndex = 0;

        sliderThreadPID = new SliderThreadPID(robot.slider);
        Thread sliderThread = new Thread(sliderThreadPID);
        sliderThread.start();
        oldSliderPower = 0;
        newSliderPower = 0;

        clawSystem = new ClawSystem(robot.claw, robot.flipper);
        clawSystem.Initial();
        clawSystem.raiseFlipper();

        positioningSystem = new PositioningSystem(robot.posLeft, robot.posRight);
        positioningSystem.Initial();

        foundationSystem = new FoundationSystem(robot.foundationLeft, robot.foundationRight);
        foundationSystem.Detach();
    }


    @Override
    public void loop() {
        super.loop();

        //Placed this here so that it is the first thing executed
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

        //Second in importance
        robot.ExpansionHub1BulkData = robot.ExpansionHub1.getBulkInputData();
        robot.ExpansionHub2BulkData = robot.ExpansionHub2.getBulkInputData();

        if (controllerInputA.leftBumperOnce() && powerCoeff > 0.2) {
            powerCoeff -= 0.1;
        } else if (controllerInputA.rightBumperOnce() && powerCoeff < 1) {
            powerCoeff += 0.1;
        }
        powerCoeff = Range.clip(powerCoeff, 0.2, 1.0);

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
        newLeftBumper = gamepad2.left_bumper;
        newRightBumper = gamepad2.right_bumper;
        if ((newLeftBumper != oldLeftBumper) || (newRightBumper != oldRightBumper)) {
            if ((!newLeftBumper && !newRightBumper))
                newSliderPower = 0;

            //move the slider forward
            if(!newLeftBumper && newRightBumper)
                newSliderPower = 1;

            //only move the slider backwards if we are more than 15 ticks
            if(newLeftBumper && !newRightBumper && robot.ExpansionHub1BulkData.getMotorCurrentPosition(robot.slider) >=20)
                newSliderPower = -1;
        }

        if(newSliderPower != oldSliderPower) {
            robot.slider.setPower(newSliderPower);
        }
        oldSliderPower = newSliderPower;
        oldLeftBumper = newLeftBumper;
        oldRightBumper = newRightBumper;


        //--------------------LIFTER------------------------------------

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
        currentState = LifterMethods.getStateFromTicks(robot.ExpansionHub2BulkData.getMotorCurrentPosition(robot.leftLifter));

        //-------------------SERVO CONTROL-----------------------------
        startButton2 = gamepad2.start;
        startButton1 = gamepad1.start;

        if (controllerInputA.BOnce() && !startButton1) { //  deal with the claw
            if (!clawSystem.isAttached()) {
                clawSystem.Attach(); //when attaching claw, wait a little then detach the positioning system
                positioningSystem.Detach();
            } else clawSystem.Detach();
        }

        if (controllerInputB.AOnce() && !startButton2) { // deal with the positioning system
            if (positioningSystem.isInitial())
                positioningSystem.Detach();
            else if (positioningSystem.isAttached())
                positioningSystem.Detach();
            else positioningSystem.Attach();
        }

        if(controllerInputB.BOnce() && !startButton2) {
            if(sliderIndex == -1)
                sliderIndex = 1;
            sliderThreadPID.setTicks(sliderStates[sliderIndex++ % sliderStates.length]);
        }


        if (controllerInputA.AOnce() && !startButton1 && !positioningSystem.isAttached()) { //deal with the foundation system
            if (foundationSystem.isAttached())
                foundationSystem.Detach();
            else foundationSystem.Attach();
        }

        updateTelemetry();
    }

    @Override
    public void stop() {
        super.stop();
        LifterThread.finished = true; //stop the loop
        LifterThread.kill = true; //stop the entire thread
        robot.leftLifter.setPower(0);
        robot.rightLifter.setPower(0);
    }

    private void updateTelemetry() {
        powerCoeffTelemetry.setValue(powerCoeff);
        armTelemetry.setValue("Lifter:  " + robot.ExpansionHub2BulkData.getMotorCurrentPosition(robot.leftLifter) + "  Next level:  " + levelIndex
                 + " Slider  " + robot.ExpansionHub1BulkData.getMotorCurrentPosition(robot.slider));
        telemetry.update();
    }


}