package org.firstinspires.ftc.teamcode.Misc;

/**
 * This class contains the enum that stores the position of the Lifter.
 * It has additional methods such as getNextState and getPreviousState.
 * The method getTicksFromState converts the enum value to an actual
 * encoder value. These values where determined through testing.
 */

public class LifterMethods {


    public static LIFTER[] level = {LIFTER.LOW, LIFTER.FIRST, LIFTER.SECOND, LIFTER.THIRD, LIFTER.FOURTH
            , LIFTER.FIFTH, LIFTER.SIXTH,LIFTER.SEVENTH, LIFTER.EIGHTH};

    public static LIFTER getStateFromTicks(int ticks) {
        if(ticks < 0) return LIFTER.FLOAT;
        if(ticks > 0 && ticks < getTicksFromState(LIFTER.FIRST)) return LIFTER.LOW;
        if(ticks > (getTicksFromState(LIFTER.LOW) + 100) && ticks < getTicksFromState(LIFTER.SECOND)) return LIFTER.FIRST;
        if(ticks > (getTicksFromState(LIFTER.FIRST) + 100) && ticks < getTicksFromState(LIFTER.THIRD)) return LIFTER.SECOND;
        if(ticks > (getTicksFromState(LIFTER.SECOND)+ 100) && ticks < getTicksFromState(LIFTER.FOURTH)) return LIFTER.THIRD;
        if(ticks > (getTicksFromState(LIFTER.THIRD)+ 100) && ticks < getTicksFromState(LIFTER.FIFTH)) return LIFTER.FOURTH;
        if(ticks > (getTicksFromState(LIFTER.FOURTH) + 100) && ticks < getTicksFromState(LIFTER.SIXTH)) return LIFTER.FIFTH;
        if(ticks > (getTicksFromState(LIFTER.FIFTH) + 100) && ticks < getTicksFromState(LIFTER.SEVENTH)) return LIFTER.SIXTH;
        if(ticks > (getTicksFromState(LIFTER.SIXTH) + 100) && ticks < getTicksFromState(LIFTER.SEVENTH)) return LIFTER.SEVENTH;
        if(ticks > (getTicksFromState(LIFTER.SEVENTH) + 100) && ticks < getTicksFromState(LIFTER.EIGHTH) + 100) return LIFTER.EIGHTH;
        if(ticks > getTicksFromState(LIFTER.EIGHTH) + 300)  return LIFTER.FLOAT;
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
        SEVENTH,
        EIGHTH,
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
            case SIXTH:
                return LIFTER.SEVENTH;
            case SEVENTH:
                return LIFTER.EIGHTH;
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
            case SEVENTH:
                return LIFTER.SIXTH;
            case EIGHTH:
                return LIFTER.SEVENTH;
        }
        return currentState;
    }

    public static int getTicksFromState(LIFTER currentState) {
        switch (currentState) {
            case LOW:
                return 0;
            case FIRST:
                return 760;
            case SECOND:
                return 1500;
            case THIRD:
                return 2500;
            case FOURTH:
                return 3200;
            case FIFTH:
                return 4000;
            case SIXTH:
                return 4600;
            case SEVENTH:
                return 5700;
            case EIGHTH:
                return 5990;
        }
        return 0; //code should never reach here. Yes, this function sucks...
    }
}
