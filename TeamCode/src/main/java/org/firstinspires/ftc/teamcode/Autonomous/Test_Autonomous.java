package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import org.openftc.revextensions2.ExpansionHubMotor;

@Autonomous(name = "Auto Test", group = "default")
public class Test_Autonomous extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        sleep(100);

        robot.lifter.setTargetPosition(3000);
        robot.lifter.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
        robot.lifter.setPower(1);
        while(robot.lifter.getCurrentPosition() < robot.lifter.getTargetPosition()) {
            telemetry.addData("encoder:  ",robot.lifter.getCurrentPosition());
            telemetry.update();
        }
        robot.lifter.setPower(0);

    }
}
