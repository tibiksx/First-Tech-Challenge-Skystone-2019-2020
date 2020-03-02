package org.firstinspires.ftc.teamcode.ComputerVision;

import org.firstinspires.ftc.teamcode.FieldStats;
import org.firstinspires.ftc.teamcode.Misc.Utilities;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CameraThread implements Runnable {

    private OpenCvCamera camera;
    private volatile Utilities.CAMERA_STATE state = Utilities.CAMERA_STATE.NULL;
    private boolean active;
    private static volatile boolean kill = false;

    ///////////////////////////////////////////////

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static final float rectHeight = .6f/8f;
    private static final float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;
    private static float offsetY = 0f/8f;

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};

    private static final int rows = 640;
    private static final int cols = 480;


    public CameraThread(OpenCvCamera camera) {
        this.camera = camera;
        active = true;
        kill = false;
        Utilities.CAMERA_STATE state = Utilities.CAMERA_STATE.NULL;
        valMid = -1;
        valLeft = -1;
        valRight = -1;
    }

    @Override
    public void run() {

        while (!kill) {

            if (active) {
                if (state == Utilities.CAMERA_STATE.INIT) {
                    try {
                        camera.openCameraDevice();
                    } catch (OpenCvCameraException e) {
                        try {
                            Thread.sleep(1500);
                        } catch (InterruptedException i) {
                            killThread();
                        }
                    }
                    try {
                        camera.setPipeline(new StageSwitchingPipeline());
                    } catch (OpenCvCameraException n) {
                        try {
                            Thread.sleep(1500);
                        } catch (InterruptedException i) {
                            killThread();
                        }
                    }
                }

                if (state == Utilities.CAMERA_STATE.STREAM) {
                    try {
                        camera.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);
                    } catch (OpenCvCameraException c) {
                        try {
                            Thread.sleep(1500);
                        } catch (InterruptedException i) {
                            killThread();
                        }
                    }
                }

                if (state == Utilities.CAMERA_STATE.DETECT) {

                    if (valLeft == 0 && valMid == 255 && valRight == 255) {  //left stone is a skystone
                        FieldStats.REDStones.markAsSkystone(4);
                        FieldStats.REDStones.markAsSkystone(4 - 3);
                    } else if (valLeft == 255 && valMid == 0 && valRight == 255) { //mid stone is a skystone
                        FieldStats.REDStones.markAsSkystone(5);
                        FieldStats.REDStones.markAsSkystone(5 - 3);
                    } else if (valLeft == 255 && valMid == 255 && valRight == 0) {  //right stone is a skystone
                        FieldStats.REDStones.markAsSkystone(6);
                        FieldStats.REDStones.markAsSkystone(6 - 3);
                    }
                }

                if (state == Utilities.CAMERA_STATE.KILL) {
                    killThread();
                }

                active = false;
            }
        }

    }

    private static void killThread() {
        kill = true;
    }

    public static boolean isAlive() {
        return kill;
    }

    public void setState(Utilities.CAMERA_STATE state) {
        this.state = state;
        this.active = true;
    }

    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage { //color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private StageSwitchingPipeline.Stage stageToRenderToViewport = StageSwitchingPipeline.Stage.detection;
        private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}