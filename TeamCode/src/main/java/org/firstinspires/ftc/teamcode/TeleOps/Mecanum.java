package org.firstinspires.ftc.teamcode.TeleOps;

import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.openftc.revextensions2.ExpansionHubMotor;

import com.qualcomm.robotcore.util.Range;

@TeleOp(name = " ", group = " ")
public class Mecanum extends Robot{

    final static int LOWERED_TICKS = 0;
    final static int FIRST_TICKS = 50;
    final static int SECOND_TICKS = 100;
    final static int THIRD_TICKS = 150;
    final static int HIGH_TICKS = 200;

    public enum LIFTER_STATES
    {
        LOWERED,
        FIRST,
        SECOND,
        THIRD,
        HIGH

    }

    public long currTimeMillis = 0; //time in ms

     public LIFTER_STATES lifter_state;

    @Override
    public void init() {
        super.init();
        lifter_state = LIFTER_STATES.LOWERED;
    }

    @Override
    public void init_loop(){
        super.init_loop();
    }

    @Override
    public void start(){
        super.start();
    }

    @Override
    public void loop() {
        currTimeMillis = SystemClock.uptimeMillis();

        float drive, turn, strafe;

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rightBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

        robot.fl.setPower(leftFrontPower);
        robot.fr.setPower(rightFrontPower);
        robot.bl.setPower(leftBackPower);
        robot.br.setPower(rightBackPower);

        //Update controller custom wrapper
        controllerInputA.update();
        controllerInputB.update();


        if(controllerInputA.XOnce()) //if x is pressed once
        {
            //move to higher state
            LIFTER_STATES newState = nextState();
            goToState(newState);

        }
        if(controllerInputA.AOnce()) //if a is pressed once
        {
            //move to lower state
            LIFTER_STATES newState =  previousState();
            goToState(newState);
        }
        telemetry.addData("Lifter Ticks:  ",robot.Lifter.getCurrentPosition());

    }

    LIFTER_STATES previousState() {
        switch (lifter_state)
        {
            case FIRST: lifter_state = LIFTER_STATES.LOWERED; break;
            case SECOND: lifter_state = LIFTER_STATES.FIRST; break;
            case THIRD: lifter_state = LIFTER_STATES.SECOND; break;
            case HIGH: lifter_state = LIFTER_STATES.THIRD;  break;
        }

        return lifter_state;
    }


    LIFTER_STATES nextState()
    {
        switch (lifter_state)
        {
            case LOWERED: lifter_state = LIFTER_STATES.FIRST; break;
            case FIRST: lifter_state = LIFTER_STATES.SECOND; break;
            case SECOND: lifter_state = LIFTER_STATES.THIRD; break;
            case THIRD: lifter_state = LIFTER_STATES.HIGH; break;
        }

        return lifter_state;
    }


    public void goToState(LIFTER_STATES newState) {
        int position = 0;
        if(newState == LIFTER_STATES.LOWERED) position = LOWERED_TICKS;
        if(newState == LIFTER_STATES.FIRST) position = FIRST_TICKS;
        if(newState == LIFTER_STATES.SECOND) position = SECOND_TICKS;
        if(newState == LIFTER_STATES.THIRD) position = THIRD_TICKS;
        if(newState == LIFTER_STATES.HIGH) position = HIGH_TICKS;

        robot.Lifter.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
        robot.Lifter.setTargetPosition(position);

        robot.Lifter.setPower(1);
        while (robot.Lifter.isBusy()) {

        }
        robot.Lifter.setPower(0);
        robot.Lifter.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
    }
}
