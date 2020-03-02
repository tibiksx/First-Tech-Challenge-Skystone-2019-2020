package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ComputerVision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.FieldStats;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Misc.*;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;
import org.firstinspires.ftc.teamcode.Threads.SliderThreadPID;
import org.opencv.core.Point;
import org.openftc.revextensions2.ExpansionHubMotor;

@Autonomous(name = "Auto BLUE", group = "Autonomous")
public class Autonomous_BLUE extends LinearOpMode {

    private Hardware robot = new Hardware();
    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    private LifterThread lifterThread = null;
    public LifterMethods.LIFTER currentState;
    private int levelIndex;

    private ClawSystem clawSystem = null;
    private PositioningSystem positioningSystem = null;
    private FoundationSystem foundationSystem = null;

    private SkystoneDetector skystoneDetector;

    private final double initialXCoordinate = 0;
    private final double initialYCoordinate = 0;
    private final double initialOrientationDegrees = 0;

    private long currentMillis;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initIMU();
        robot.initWebcam();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal,robot.ExpansionHub1, Utilities.TICKS_PER_INCH, 75);
        globalPositionUpdate.setInitialCoordinates(initialXCoordinate, initialYCoordinate, Math.toRadians(initialOrientationDegrees));
        globalPositionUpdate.reverseRightEncoder();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        clawSystem = new ClawSystem(robot.claw, robot.flipper);
        clawSystem.Initial();
        clawSystem.raiseFlipper();

        positioningSystem = new PositioningSystem(robot.posLeft, robot.posRight);
        positioningSystem.Initial();

        foundationSystem = new FoundationSystem(robot.foundationLeft,robot.foundationRight);
        foundationSystem.Detach();


        lifterThread = new LifterThread(robot.leftLifter, robot.rightLifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();
        currentState = LifterMethods.LIFTER.FLOAT;
        levelIndex = 0;

        SliderThreadPID sliderThreadPID = new SliderThreadPID(robot.slider);
        Thread sliderThread = new Thread(sliderThreadPID);
        sliderThread.start();

        skystoneDetector = new SkystoneDetector(robot.webcam,telemetry,robot.cameraMonitorViewId);
        skystoneDetector.init();

        waitForStart();
        int[] stoneValues = skystoneDetector.scan();
        int valLeft = stoneValues[0];
        int valMid = stoneValues[1];
        int valRight = stoneValues[2];
        skystoneDetector.killCamera();

        if (valLeft == 0 && valMid == 255 && valRight == 255) {  //left stone is a skystone
            FieldStats.BLUEStones.markAsSkystone(4);
            FieldStats.BLUEStones.markAsSkystone(4 - 3);
        } else if (valLeft == 255 && valMid == 0 && valRight == 255) { //mid stone is a skystone
            FieldStats.BLUEStones.markAsSkystone(5);
            FieldStats.BLUEStones.markAsSkystone(5 - 3);
        } else if (valLeft == 255 && valMid == 255 && valRight == 0) {  //right stone is a skystone
            FieldStats.BLUEStones.markAsSkystone(6);
            FieldStats.BLUEStones.markAsSkystone(6 - 3);
        }

        FieldStats.BLUEStones.displayPattern(telemetry);

        //find the closest stone to pick up
        FieldStats.Stone closestStone1 = new FieldStats.Stone(new Point(0,0),false);
        for(int i = 6; i>=1; i--) {
            if(FieldStats.BLUEStones.stoneArray[i].isSkystone()) {
                closestStone1 = FieldStats.BLUEStones.stoneArray[i];
                break;
            }
        }
        //only if we found a skystone
//        if(!(closestStone1.getPosition().x == 0 && closestStone1.getPosition().y == 0)) {
//
//            //move slider forward
//            sliderThreadPID.setTicks(1200);
//            //go to the stone's position
//            Utilities.goToPositionNoTurning(closestStone1.getPosition().x, closestStone1.getPosition().y, 0.7, globalPositionUpdate, robot,telemetry);
//            sleep(500);
//            //catch it
//            clawSystem.Attach();
//            sleep(500);
//            //pull it back
//            sliderThreadPID.setTicks(670);
//            //back off a little
//            Utilities.goToPositionNoTurning(closestStone1.getPosition().x-20, closestStone1.getPosition().y, 0.7, globalPositionUpdate, robot,telemetry);
//            //TODO Find a way to rotate
//
//        }

    }

    private void moveSlider(int position) {
        robot.slider.setTargetPosition(position);
        robot.slider.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
        if (robot.slider.getCurrentPosition() < position) {
            robot.slider.setPower(1);
            while (robot.slider.getCurrentPosition() < position - 100) {
                telemetry.addData("POZ", robot.slider.getPower() + " " + robot.slider.getCurrentPosition());
                telemetry.update();
            }
        } else {
            robot.slider.setPower(-1);
            while (robot.slider.getCurrentPosition() > position + 100) {
                telemetry.addData("POZ", robot.slider.getPower() + " " + robot.slider.getCurrentPosition());
                telemetry.update();
            }
        }
        robot.slider.setPower(0);
        sleep(50);
    }

}
