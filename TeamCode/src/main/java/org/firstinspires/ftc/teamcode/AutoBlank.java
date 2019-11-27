package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="AutoBlank", group="Pushbot")
public class AutoBlank extends OpMode {

    Hardware robot = new Hardware();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        telemetry.log().clear();
        telemetry.addData("Currently in:", "INIT");
        telemetry.update();

        robot.init(hardwareMap);

    }

    // After init() function, execute in loop until op mode is started
    @Override
    public void init_loop() {
    }

    // Execute ONCE when the op mode is started
    @Override
    public void start() {


    }

    // Execute until driver hits STOP
    @Override
    public void loop() {


    }

    // When driver hits STOP, happens once
    @Override
    public void stop() {

    }
}
