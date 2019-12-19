package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Misc.Encoder;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.io.File;


@Disabled
public class GlobalPositionThread implements Runnable {


    private Encoder leftEncoder = null;
    private Encoder rightEncoder = null;
    private Encoder backEncoder = null;

    public static volatile boolean kill = false;

    private double robotGlobalXCoordinate = 0;
    private double robotGlobalYCoordinate = 0;
    private double robotOrientation = 0;     // IN RADIANS

    private double backEncoderPosition = 0;
    private double leftEncoderPosition = 0;
    private double rightEncoderPosition = 0;

    private double deltaOrientation = 0;
    private double prevBackEncoderPosition = 0;
    private double prevLefEncoderPosition = 0;
    private double prevRightEncoderPosition = 0;

    private int backEncoderSign = 1;
    private int leftEncoderSign = 1;
    private int rightEncoderSign = 1;

    private double wheelSeparation;
    private double backEncoderOffsetPerDeg;

    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelsData.txt");
    private File backEncoderOffsetFile = AppUtil.getInstance().getSettingsFile("backTicksFile.txt");

    public GlobalPositionThread(Encoder backEncoder, Encoder leftEncoder, Encoder rightEncoder, double COUNTS_PER_INCH)
    {
        this.backEncoder = backEncoder;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;

        this.wheelSeparation = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.backEncoderOffsetPerDeg = Double.parseDouble(ReadWriteFile.readFile(backEncoderOffsetFile).trim());
    }


    private void PositionUpdate()
    {
        backEncoderPosition = backEncoder.getPosition() * backEncoderSign;
        leftEncoderPosition = leftEncoder.getPosition() * leftEncoderSign;
        rightEncoderPosition = rightEncoder.getPosition() * rightEncoderSign;

        double deltaLeftPosition = leftEncoderPosition - prevLefEncoderPosition;
        double deltaRightPosition = rightEncoderPosition - prevRightEncoderPosition;

        deltaOrientation = (deltaLeftPosition - deltaRightPosition)/(wheelSeparation);
        robotOrientation = robotOrientation + deltaOrientation;

        double deltaBackPosition = (backEncoderPosition - prevBackEncoderPosition) - (deltaOrientation * backEncoderOffsetPerDeg);

        double deltaCenterPosition = (deltaRightPosition + deltaLeftPosition)/2;

        robotGlobalXCoordinate += (deltaCenterPosition * Math.sin(robotOrientation) +
                deltaBackPosition * Math.cos(robotOrientation));

        robotGlobalYCoordinate += (deltaCenterPosition * Math.cos(robotOrientation)) +
                deltaBackPosition * Math.sin(robotOrientation);

        prevBackEncoderPosition = backEncoderPosition;
        prevRightEncoderPosition = rightEncoderPosition;
        prevLefEncoderPosition = leftEncoderPosition;
    }

    public double getGlobalX()
    {
        return robotGlobalXCoordinate;
    }

    public double getGlobalY()
    {
        return robotGlobalYCoordinate;
    }

    public double getAngleInDeg()
    {
        return Math.toDegrees(robotOrientation) % 360;
    }

    public double getAngleInRad()
    {
        return robotOrientation;
    }


    @Override
    public void run() {
        while(!kill)
        {
            PositionUpdate();
            try
            {
                Thread.sleep(100);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
    }

    public void reverseLeftEncoder()
    {
        leftEncoderSign *= -1;
    }

    public void reverseRightEncoder()
    {
        rightEncoderSign *= -1;
    }

    public void reverseBackEncoder()
    {
        backEncoderSign *= -1;
    }
}