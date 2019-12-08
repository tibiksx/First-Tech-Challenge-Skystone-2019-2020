package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import org.openftc.revextensions2.ExpansionHubMotor;

@Autonomous(name = "Auto Test", group = "default")
public class Test_Autonomous extends LinearOpMode {
    Hardware robot = new Hardware();

    final static double NEW_P = 15;
    final static double NEW_I = 0;
    final static double NEW_D = 0;
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        sleep(100);

        /*PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        robot.lifter.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
        PIDCoefficients pidModified = robot.lifter.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifter.setTargetPosition(3000);
        robot.lifter.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
        robot.lifter.setPower(1);
        while(robot.lifter.getCurrentPosition() < robot.lifter.getTargetPosition()) {
            telemetry.addData("p:",pidModified.p);
            telemetry.addData("i:",pidModified.i);
            telemetry.addData("d:",pidModified.d);
            telemetry.update();
        }
        robot.lifter.setPower(0); */

        robot.lifter.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        robot.lifter.setPower(1);
        while(robot.lifter.getCurrentPosition() < 3000) {
            telemetry.addData("encoder:",robot.lifter.getCurrentPosition());
            telemetry.update();
        }
        robot.lifter.setPower(0);

    }
}
