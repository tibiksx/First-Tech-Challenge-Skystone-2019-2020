package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.io.File;

/**
 * Sample created by Sarthak on 6/1/2019.
 * Modified and implemented by Tiberiu Ureche on 12/01/2020.
 */
public class OdometryGlobalCoordinatePosition implements Runnable{

    private ExpansionHubMotor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;

    public boolean isRunning = true;

    public double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    public double robotGlobalXCoordinatePosition = 0.01, robotGlobalYCoordinatePosition = 0.01, robotOrientationRadians = 0.01;
    public double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    public double robotEncoderWheelDistance;
    public double horizontalEncoderTickPerDegreeOffset;

    public int sleepTime;

    public File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    public File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    public int verticalLeftEncoderPositionMultiplier = 1;
    public int verticalRightEncoderPositionMultiplier = 1;
    public int normalEncoderPositionMultiplier = 1;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalCoordinatePosition(ExpansionHubMotor verticalEncoderLeft, ExpansionHubMotor verticalEncoderRight, ExpansionHubMotor horizontalEncoder, double COUNTS_PER_INCH, int threadSleepDelay){
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;
        sleepTime = threadSleepDelay;

        //robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        robotEncoderWheelDistance = 6.5 * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    public void globalCoordinatePositionUpdate(){
        //Get Current Positions
        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

        //Get the components of the motion
        normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return robotGlobalXCoordinatePosition; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return robotGlobalYCoordinatePosition; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientationDeg(){ return Math.toDegrees(robotOrientationRadians) % 360; }

    public double returnOrientationRad() { return robotOrientationRadians; }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    public void reverseLeftEncoder(){
        if(verticalLeftEncoderPositionMultiplier == 1){
            verticalLeftEncoderPositionMultiplier = -1;
        }else{
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder(){
        if(verticalRightEncoderPositionMultiplier == 1){
            verticalRightEncoderPositionMultiplier = -1;
        }else{
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseNormalEncoder(){
        if(normalEncoderPositionMultiplier == 1){
            normalEncoderPositionMultiplier = -1;
        }else{
            normalEncoderPositionMultiplier = 1;
        }
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

