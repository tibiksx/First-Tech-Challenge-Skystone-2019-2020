package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubMotor;

/**
 * This class is the wrapper for every Encoder. It contains information
 * about an encoder: the corresponding motor, the current position, the
 * number of rotations and the distance in centimeters. These last three
 * values are update by calling updateEncoder() in loop.
 */

public class Encoder {

    private ExpansionHubMotor encoderMotor;
    private volatile int encoderPosition = 0;
    private volatile double encoderRotations = 0;
    private volatile double encoderDistance = 0;

    public Encoder(ExpansionHubMotor encoderMotor) {
        this.encoderMotor = encoderMotor;
    }

    public void updateEncoder() {

        encoderPosition = encoderMotor.getCurrentPosition();
        encoderRotations = encoderPosition / Constants.TICKS_PER_REV;
        encoderDistance = Math.PI * Constants.ENCODER_WHEEL_DIAMETER * encoderRotations;
    }

    public int getPosition() {
        return encoderPosition;
    }

    public double getRotations() { return encoderRotations; }

    public void resetTicks() {
        encoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderMotor.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getDistance() {
        return encoderDistance;
    }
}
