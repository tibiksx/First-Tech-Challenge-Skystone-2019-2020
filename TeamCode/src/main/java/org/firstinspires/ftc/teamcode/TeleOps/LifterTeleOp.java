package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.LifterMethods;
import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;
import org.openftc.revextensions2.ExpansionHubMotor;

import static org.firstinspires.ftc.teamcode.Misc.LifterMethods.level;

/**
 * This class includes the lifter and the slider arm. Threads also
 */

@TeleOp(name = "Lifter + Extension Arm Only TeleOp", group = "Test TeleOp")
public class LifterTeleOp extends Robot {

    public LifterThread lifterThread;

    public LifterMethods.LIFTER currentState;

    private double lifterOldPower = 0;
    private double lifterNewPower = 0;
    private int levelIndex = 0;

    private boolean leftBumperInput;
    private boolean rightBumperInput;

    private boolean buttonIsPressed = false;

    @Override
    public void init() {
        super.init();

        lifterThread = new LifterThread(robot.leftLifter, robot.rightLifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();

        currentState = LifterMethods.LIFTER.LOW;
    }

    @Override
    public void loop() {
        super.loop();

        //-----------------SLIDE ARM----------------------------
        leftBumperInput = gamepad2.left_bumper;
        rightBumperInput = gamepad2.right_bumper;
        if (!leftBumperInput && !rightBumperInput)
            robot.slider.setPower(0);
        else if (leftBumperInput && !rightBumperInput)
            robot.slider.setPower(-1);
        else if (!leftBumperInput)
            robot.slider.setPower(1);

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

        if ((controllerInputB.dpadUpOnce() || controllerInputB.dpadDownOnce()) && LifterThread.finished) {

            lifterThread.setTicks(LifterMethods.getTicksFromState(level[levelIndex]));
            currentState = level[levelIndex];
        }

        currentState = LifterMethods.getStateFromTicks(robot.leftLifter.getCurrentPosition());

        telemetry.addData("Lifter State: ", currentState + "  Next:" + levelIndex +
                +robot.leftLifter.getCurrentPosition());
        telemetry.update();

    }
}
