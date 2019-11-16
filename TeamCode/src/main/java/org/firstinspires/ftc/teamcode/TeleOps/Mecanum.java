package org.firstinspires.ftc.teamcode.TeleOps;

import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Misc.Robot;


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

        //Update controller custom wrapper
        controllerInputA.update();
        controllerInputB.update();


        if(controllerInputA.XOnce()) //if x is pressed once
        {
            //move to higher state
            LIFTER_STATES newState = nextState();
            goToState(newState);

        }
        telemetry.addData("Lifter Ticks:  ",robot.Lifter.getCurrentPosition());



    }



    LIFTER_STATES nextState()
    {
        switch (lifter_state)
        {
            case FIRST: lifter_state = LIFTER_STATES.SECOND;
            case SECOND: lifter_state = LIFTER_STATES.THIRD;
            case THIRD: lifter_state = LIFTER_STATES.HIGH;
            case HIGH: lifter_state = LIFTER_STATES.HIGH;
        }

        return lifter_state;
    }

    void goToSecond()
    {
        robot.Lifter.setTargetPosition(SECOND_TICKS);
        robot.Lifter.setPower(0.5);

    }
    public void goToState(LIFTER_STATES newState) {
        int position = 0;
        if(newState == LIFTER_STATES.LOWERED) position = LOWERED_TICKS;
        if(newState == LIFTER_STATES.FIRST) position = FIRST_TICKS;
        if(newState == LIFTER_STATES.SECOND) position = SECOND_TICKS;
        if(newState == LIFTER_STATES.THIRD) position = THIRD_TICKS;
        if(newState == LIFTER_STATES.HIGH) position = HIGH_TICKS;

        robot.Lifter.setTargetPosition(position);
        robot.Lifter.setPower(0.5);

    }
}
