package org.firstinspires.ftc.teamcode.Misc;

import android.os.SystemClock;
import java.util.ArrayList;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubServo;

public class Robot extends OpMode {

    public static boolean usingDebugger = false; //for debugging using UDP Unicast

    //--------------REV STUFF-------------------------
    //private RevBulkData revExpansionBulkData;
    ExpansionHubEx expansionHub;
    //------------------------------------------------

    public long currTimeMillis = 0; //time in ms

    private DriveTrain driveTrain;
    //holds all the rev expansion hub motors
    private ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();

    @Override
    public void init() {
        currTimeMillis = SystemClock.uptimeMillis();


        //---------------REV MOTORS--------------------
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub");
        ExpansionHubMotor fl =(ExpansionHubMotor)hardwareMap.get("fl");
        ExpansionHubMotor fr = (ExpansionHubMotor)hardwareMap.get("fr");
        ExpansionHubMotor bl = (ExpansionHubMotor)hardwareMap.get("bl");
        ExpansionHubMotor br = (ExpansionHubMotor)hardwareMap.get("br");

        allMotors.add(fl);
        allMotors.add(fr);
        allMotors.add(bl);
        allMotors.add(br);
        driveTrain = new DriveTrain();
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        currTimeMillis = SystemClock.uptimeMillis();
        //WIP
    }


}
