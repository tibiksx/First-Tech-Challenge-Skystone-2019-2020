package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import org.firstinspires.ftc.teamcode.Misc.DifferentialDrive;
import org.openftc.revextensions2.ExpansionHubMotor;

@Autonomous(name = "Auto Test", group = "default")
public class Test_Autonomous extends LinearOpMode {
    Hardware robot = new Hardware();
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        sleep(100);

        double leftSpeed = DifferentialDrive.getDifferentialPower(20);
        robot.frontLeftWheel.setPower(leftSpeed);
        robot.backLeftWheel.setPower(leftSpeed);
        robot.frontRightWheel.setPower(1);
        robot.backRightWheel.setPower(1);


    }
}
