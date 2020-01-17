package org.firstinspires.ftc.teamcode.Misc;

/**
 * This class contains the enum that stores the position of the Lifter.
 * It has additional methods such as getNextState and getPreviousState.
 * The method getTicksFromState converts the enum value to an actual
 * encoder value. These values where determined through testing.
 */
public class LifterMethods {

    public enum LIFTER {
        LOW,
        FIRST,
        SECOND,
        THIRD,
        FOURTH,
        FIFTH,
        SIXTH
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
        return -1; //code should never reach here. Yes, this function sucks...
    }
}
