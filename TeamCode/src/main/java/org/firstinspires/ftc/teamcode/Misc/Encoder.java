package org.firstinspires.ftc.teamcode.Misc;

import org.openftc.revextensions2.ExpansionHubMotor;

public class Encoder {

    private ExpansionHubMotor encoderMotor;
    double ticksPerRev;
    int wheelDiameter = -1;
    public Encoder(ExpansionHubMotor encoderMotor, double ticksPerRev,int wheelDiameter)
    {
        this.encoderMotor = encoderMotor;
        this.ticksPerRev = ticksPerRev;
        encoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        this.wheelDiameter = wheelDiameter;
    }

    public Encoder(ExpansionHubMotor encoderMotor, double ticksPerRev)
    {
        this.encoderMotor = encoderMotor;
        this.ticksPerRev = ticksPerRev;
        encoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getTicks()
    {
        return encoderMotor.getCurrentPosition();
    }

    public double getRotations()
    {
        return encoderMotor.getCurrentPosition()/(float)ticksPerRev;
    }

    public void resetTicks()
    {
        encoderMotor.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getDistance()
    {
        if(wheelDiameter == -1) return -1;
        return (wheelDiameter * getRotations());
    }
}
