package org.firstinspires.ftc.teamcode.TeleOps;

import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.ControllerInput;
import org.firstinspires.ftc.teamcode.Misc.Encoder;
import org.firstinspires.ftc.teamcode.Misc.Robot;
import org.firstinspires.ftc.teamcode.Misc.UDP_Unicast_Server;
import org.firstinspires.ftc.teamcode.Threads.MotorThread;
import org.openftc.revextensions2.ExpansionHubMotor;

import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test_TeleOp", group = "Pushbot")
public class Mecanum extends Robot{

    public long currTimeMillis = 0; //time in ms
    final public float powerFraction = (float)0.6;

    double posFound1 = 0.0, posFound2 = 0.0;

    public UDP_Unicast_Server udpServer = null;

    public Encoder rightEncoder = null;
    public Encoder leftEncoder = null;
    public Encoder backEncoder = null;

    private final int ticksPerRev = 1600;
    private final int wheelDiameter = 10; //in cm

    @Override
    public void init() {
        super.init();
        if(usingDebugger) udpServer = new UDP_Unicast_Server(50000);
        //rightEncoder = new Encoder(robot.rightEncoderMotor,ticksPerRev);
        rightEncoder = new Encoder(robot.rightEncoderMotor,ticksPerRev);

    }

    @Override
    public void init_loop(){
        super.init_loop();
    }

    @Override
    public void start(){
        super.start();
    }

    @Override
    public void loop() {
        float drive, turn, strafe;

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        // clip() = demands a number to be in certain bounds
        // number is calculated and then processed
        double leftFrontPower = //parsePower
                (Range.clip(drive + turn - strafe, -1.0, 1.0));
        double leftBackPower = //parsePower
                (Range.clip(drive + turn + strafe, -1.0, 1.0));
        double rightFrontPower = //parsePower
                (Range.clip(drive - turn + strafe, -1.0, 1.0));
        double rightBackPower = //parsePower
                (Range.clip(drive - turn - strafe, -1.0, 1.0));

        robot.frontLeftWheel.setPower(powerFraction*leftFrontPower);
        robot.frontRightWheel.setPower(powerFraction*rightFrontPower);
        robot.backLeftWheel.setPower(powerFraction*leftBackPower);
        robot.backRightWheel.setPower(powerFraction*rightBackPower);

        robot.lifter.setPower(gamepad2.right_stick_y);

        if (gamepad2.a) {
            posFound1 = 1;
        }

        if (gamepad2.b) {
            posFound1 = 0.3;
        }

        if (gamepad2.x) {
            posFound2 = 0;
        }

        if (gamepad2.y) {
            posFound2 = 0.55;
        }

        if (gamepad1.a) {
            robot.fliper2.setPosition(1);
        }

        if (gamepad1.b) {
            robot.fliper2.setPosition(0);
        }

        if (gamepad1.x) {
            robot.fliper1.setPosition(0.90);
        }

        if (gamepad1.y) {
            robot.fliper1.setPosition(0.0);
        }
       /* if(controllerInputB.dpadUpOnce())
        {
            Thread runner = new Thread(new MotorThread(robot.lifter,1000,1));
            runner.start();
            try {
                runner.join();
            }
            catch (InterruptedException e)
            {
                telemetry.addData("Failed to join ",runner.getId());
            }

        } */

        robot.slider.setPower(-gamepad2.left_stick_y);

        robot.foundation1.setPosition(posFound1);
        robot.foundation2.setPosition(posFound2);

        //telemetry.addData("Servo Fundatie 1:", robot.foundation1.getPosition());
        //telemetry.addData("Servo Fundatie 2:", robot.foundation2.getPosition());
        //computerDebugging.markEndOfUpdate();
        telemetry.addData("encoder",rightEncoder.getDistance());
        telemetry.update();

    }

    // When driver hits STOP, happens once
    @Override
    public void stop() {

        telemetry.log().clear();
        telemetry.addData("Currently in:", "STOP");
        telemetry.update();
    }

    double parsePower(double power)
    {
        if(power<0) return power;
        return (1.04 - 1.068 * Math.pow(Math.E,-8.92*power));
    }

}