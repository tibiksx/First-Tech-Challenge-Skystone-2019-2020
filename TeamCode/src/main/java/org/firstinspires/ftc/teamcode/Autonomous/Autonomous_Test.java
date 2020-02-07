package org.firstinspires.ftc.teamcode.Autonomous;

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

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, Utilities.TICKS_PER_INCH, 75);
        globalPositionUpdate.setInitialCoordinates(initialXCoordinate,initialYCoordinate,Math.toRadians(initialOrientationDegrees));
        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        clawSystem = new ClawSystem(robot.clawLeft, robot.clawRight, robot.flipper);
        clawSystem.Detach();
        sleep(100);
        clawSystem.lowerFlipper();
        sleep(100);

        positioningSystem = new PositioningSystem(robot.posLeft, robot.posRight);
        positioningSystem.Initial();


        lifterThread = new LifterThread(robot.leftLifter, robot.rightLifter);
        Thread lifterRunner = new Thread(lifterThread);
        lifterRunner.start();
        currentState = LifterMethods.LIFTER.FLOAT;
        levelIndex = 0;

        waitForStart();

        moveSlider(1000);
        sleep(1000);
        clawSystem.raiseFlipper();
        positioningSystem.Detach();
        sleep(1000);
        moveSlider(3000);
        sleep(1000);
        clawSystem.lowerFlipper();
        sleep(1000);
        moveSlider(1000);
        sleep(1000);
        clawSystem.raiseFlipper();
        sleep(1000);
        positioningSystem.Attach();
        sleep(1000);
        positioningSystem.Detach();
        sleep(1000);
        clawSystem.Attach();
        sleep(1000);
        lifterThread.setTicks(LifterMethods.getTicksFromState(LifterMethods.LIFTER.FIFTH));
        while (!LifterThread.finished) {
            telemetry.addData("Thread: ", LifterThread.finished);
            telemetry.update();
        }
        sleep(300);
        telemetry.log().clear();
        lifterThread.setTicks(LifterMethods.getTicksFromState(LifterMethods.LIFTER.FIRST));
        while (!LifterThread.finished) {
            telemetry.addData("Thread: ", LifterThread.finished);
            telemetry.update();
        }
        sleep(300);

    }

    public void collectStone() {

    }

    private void moveSlider(int position) {
        robot.slider.setTargetPosition(position);
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
