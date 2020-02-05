package org.firstinspires.ftc.teamcode.Misc;

/**
 * This class contains the enum that stores the position of the Lifter.
 * It has additional methods such as getNextState and getPreviousState.
 * The method getTicksFromState converts the enum value to an actual
 * encoder value. These values where determined through testing.
 */

public class LifterMethods {


    public static LIFTER[] level = {LIFTER.LOW, LIFTER.FIRST, LIFTER.SECOND, LIFTER.THIRD, LIFTER.FOURTH, LIFTER.FIFTH, LIFTER.SIXTH};

    public static LIFTER getStateFromTicks(int ticks) {
        if(ticks < 0) return LIFTER.FLOAT;
        if(ticks > 0 && ticks < getTicksFromState(LIFTER.FIRST)) return LIFTER.LOW;
        if(ticks > getTicksFromState(LIFTER.LOW) && ticks < getTicksFromState(LIFTER.SECOND)) return LIFTER.FIRST;
        if(ticks > getTicksFromState(LIFTER.FIRST) && ticks < getTicksFromState(LIFTER.THIRD)) return LIFTER.SECOND;
        if(ticks > getTicksFromState(LIFTER.SECOND) && ticks < getTicksFromState(LIFTER.FOURTH)) return LIFTER.THIRD;
        if(ticks > getTicksFromState(LIFTER.THIRD) && ticks < getTicksFromState(LIFTER.FIFTH)) return LIFTER.FOURTH;
        if(ticks > getTicksFromState(LIFTER.FOURTH) && ticks < getTicksFromState(LIFTER.SIXTH)) return LIFTER.FIFTH;
        if(ticks > getTicksFromState(LIFTER.FIFTH) && ticks < getTicksFromState(LIFTER.SIXTH) + 300) return LIFTER.SIXTH;
        if(ticks > getTicksFromState(LIFTER.SIXTH) + 300)  return LIFTER.FLOAT;
        return LIFTER.FLOAT;
    }

    public enum LIFTER {
        LOW,
        FIRST,
        SECOND,
        THIRD,
        FOURTH,
        FIFTH,
        SIXTH,
        FLOAT
    }

    public static LIFTER getNextState(LIFTER currentState) {
        switch (currentState) {
            case LOW:
                return LIFTER.FIRST;
            case FIRST:
                return LIFTER.SECOND;
            case SECOND:
                return LIFTER.THIRD;
            case THIRD:
                return LIFTER.FOURTH;
            case FOURTH:
                return LIFTER.FIFTH;
            case FIFTH:
                return LIFTER.SIXTH;
        }
        return currentState;
    }
    public static LIFTER getPreviousState(LIFTER currentState) {
        switch (currentState) {
            case FIRST:
                return LIFTER.LOW;
            case SECOND:
                return LIFTER.FIRST;
            case THIRD:
                return LIFTER.SECOND;
            case FOURTH:
                return LIFTER.THIRD;
            case FIFTH:
                return LIFTER.FOURTH;
            case SIXTH:
                return LIFTER.FIFTH;
        }
        return currentState;
    }

    public static int getTicksFromState(LIFTER currentState) {
        switch (currentState) {
            case LOW:
                return 0;
            case FIRST:
                return 245;
            case SECOND:
                return 1080;
            case THIRD:
                return 2080;
            case FOURTH:
                return 3280;
            case FIFTH:
                return 4300;
            case SIXTH:
                return 5880;
        }
        return 0; //code should never reach here. Yes, this function sucks...
    }
}
