package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Misc.*;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Threads.LifterThread;
import org.openftc.revextensions2.ExpansionHubMotor;


import static org.firstinspires.ftc.teamcode.Misc.Utilities.goToPosition;

@Autonomous(name = "Auto Test", group = "Autonomous")
public class Autonomous_Test extends LinearOpMode {

    private Hardware robot = new Hardware();
    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    private LifterThread lifterThread = null;
    public LifterMethods.LIFTER currentState;
    private int levelIndex;

    private ClawSystem clawSystem = null;
    private PositioningSystem positioningSystem = null;

    private final double initialXCoordinate = 0;
    private final double initialYCoordinate = 0;
    private final double initialOrientationDegrees = 0;

    private long currentMillis;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, Utilities.TICKS_PER_INCH, 75);
        globalPositionUpdate.setInitialCoordinates(initialXCoordinate,initialYCoordinate,Math.toRadians(initialOrientationDegrees));
        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        clawSystem = new ClawSystem(robot.claw, robot.flipper);
        clawSystem.Initial();
        clawSystem.raiseFlipper();

        positioningSystem = new PositioningSystem(robot.posLeft, robot.posRight);
        positioningSystem.Initial();


        lifterThread = new LifterThread(robot.leftLifter, robot.rightLifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();
        currentState = LifterMethods.LIFTER.FLOAT;
        levelIndex = 0;

        waitForStart();

        currentMillis = SystemClock.uptimeMillis();

        goToPosition(0,60,Math.toRadians(0),0.6,globalPositionUpdate,robot);

    }

    public void collectStone() {
        clawSystem.lowerFlipper();
        sleep(1000);
        moveSlider(1000);
        sleep(600);
        clawSystem.raiseFlipper();
        sleep(1000);
        positioningSystem.Attach();
        sleep(1000);
        positioningSystem.Detach();
        sleep(1000);
        clawSystem.Attach();
        sleep(500);
    }

    private void moveSlider(int position) {
        robot.slider.setTargetPosition(position);
        robot.slider.setTargetPositionTolerance(50);
        robot.slider.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
        if (robot.slider.getCurrentPosition() < position) {
            robot.slider.setPower(1);
            while (robot.slider.getCurrentPosition() < position-100) {
                telemetry.addData("POZ", robot.slider.getPower() + " " + robot.slider.getCurrentPosition());
                telemetry.update();
            }
        } else {
            robot.slider.setPower(-1);
            while (robot.slider.getCurrentPosition() > position+100) {
                telemetry.addData("POZ", robot.slider.getPower() + " " + robot.slider.getCurrentPosition());
                telemetry.update();
            }
        }
        robot.slider.setPower(0);
        sleep(50);
    }

}
