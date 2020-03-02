package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Misc.Utilities;
import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.openftc.revextensions2.ExpansionHubEx;

import java.text.DecimalFormat;
import java.text.NumberFormat;

@TeleOp(name = "Test TeleOp")
public class Test_TeleOp extends Robot {

    private static double xGoal = 60;
    private static double yGoal = 0;
    private static double turnSpeed = 0.7;
    private static final double preferredAngle = Math.toRadians(90);

    private OdometryGlobalCoordinatePosition globalPositionUpdate;

    private ExpansionHubEx expansionHub;

    @Override
    public void init() {
        super.init();

        robot.init(hardwareMap);

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal,robot.ExpansionHub1,Utilities.TICKS_PER_INCH, 75);

        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();

    }

    @Override
    public void loop() {
        telemetry.addData("Total current", expansionHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.update();
    }
}
