package org.firstinspires.ftc.teamcode.Misc;

import org.openftc.revextensions2.ExpansionHubMotor;

public class Encoder {

    private ExpansionHubMotor encoderMotor;
    private int encoderPosition = 0;

    public Encoder(ExpansionHubMotor encoderMotor)
    {
        this.encoderMotor = encoderMotor;
        encoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateEncoder(int encoderPosition)
    {
        this.encoderPosition = encoderPosition;
    }

    public int getPosition()
    {
        return encoderPosition;
    }

    public double getRotations()
    {
        return encoderPosition/(float)Constants.TICKS_PER_REV;
    }

    public void resetTicks()
    {
        encoderMotor.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getDistance()
    {
        return (Constants.ENCODER_WHEEL_DIAMETER * getRotations());
    }
}
