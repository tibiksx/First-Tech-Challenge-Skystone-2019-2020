package org.firstinspires.ftc.teamcode.Misc;

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
                return 500;
            case FIRST:
                return 1400;
            case SECOND:
                return 3000;
            case THIRD:
                return 4800;
            case FOURTH:
                return 6300;
            case FIFTH:
                return 8000;
            case SIXTH:
                return 9700;
        }
        return -1; //code should never reach here. Yes, this function sucks...
    }
}
