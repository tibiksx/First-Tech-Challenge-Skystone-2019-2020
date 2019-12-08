package org.firstinspires.ftc.teamcode.Misc;

import org.firstinspires.ftc.teamcode.TeleOps.Mecanum;

public class LifterMethods {

    public static Mecanum.LIFTER getNextState(Mecanum.LIFTER currentState) {
        switch (currentState) {
            case LOW:
                return Mecanum.LIFTER.FIRST;
            case FIRST:
                return Mecanum.LIFTER.SECOND;
            case SECOND:
                return Mecanum.LIFTER.THIRD;
            case THIRD:
                return Mecanum.LIFTER.FOURTH;
            case FOURTH:
                return Mecanum.LIFTER.FIFTH;
            case FIFTH:
                return Mecanum.LIFTER.SIXTH;
        }
        return currentState;
    }
    public static Mecanum.LIFTER getPreviousState(Mecanum.LIFTER currentState) {
        switch (currentState) {
            case FIRST:
                return Mecanum.LIFTER.LOW;
            case SECOND:
                return Mecanum.LIFTER.FIRST;
            case THIRD:
                return Mecanum.LIFTER.SECOND;
            case FOURTH:
                return Mecanum.LIFTER.THIRD;
            case FIFTH:
                return Mecanum.LIFTER.FOURTH;
            case SIXTH:
                return Mecanum.LIFTER.FIFTH;
        }
        return currentState;
    }

    public static int getTicksFromState(Mecanum.LIFTER currentState) {
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
