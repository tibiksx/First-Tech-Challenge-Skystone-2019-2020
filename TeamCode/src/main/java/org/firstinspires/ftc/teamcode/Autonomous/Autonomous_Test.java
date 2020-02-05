package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Misc.Utilities;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import static org.firstinspires.ftc.teamcode.Misc.Utilities.goToPosition;

@Autonomous(name = "Auto Test", group = "Autonomous")
public class Autonomous_Test extends LinearOpMode {

    Hardware robot = new Hardware();
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, Utilities.TICKS_PER_INCH, 75);

        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        waitForStart();

        goToPosition(60,60,Math.toRadians(0),0.6,globalPositionUpdate,robot);
        sleep(600);
        goToPosition(0,0,Math.toRadians(0),0.6,globalPositionUpdate,robot);
        sleep(600);


    }

}
