package org.firstinspires.ftc.teamcode.Misc;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubEx;

public class Robot extends OpMode {

    public static boolean usingDebugger = false; //for debugging using UDP Unicast

    //--------------REV STUFF-------------------------
    private RevBulkData revExpansionMasterBulkData;
    private RevBulkData getRevExpansionSlaveBulkData;
    ExpansionHubEx revMaster;
    ExpansionHubEx revSlave;
    //------------------------------------------------



    //--------------------REV MOTORS-------------------
    ExpansionHubMotor fl = null;
    ExpansionHubMotor fr = null;
    ExpansionHubMotor bl = null;
    ExpansionHubMotor br = null;
    ExpansionHubMotor Lifter = null;
    //-----------------------------------------------

    public long currTimeMillis = 0; //time in ms

    private DriveTrain driveTrain;


    @Override
    public void init() {
        currTimeMillis = SystemClock.uptimeMillis();


        //---------------REV MOTORS--------------------
        revMaster = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        revSlave = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");
        fl =(ExpansionHubMotor)hardwareMap.get("FL");
        fr = (ExpansionHubMotor)hardwareMap.get("FR");
        bl = (ExpansionHubMotor)hardwareMap.get("BL");
        br = (ExpansionHubMotor)hardwareMap.get("BR");
        Lifter = (ExpansionHubMotor)hardwareMap.get("Lifter");

        driveTrain = new DriveTrain(fr,fl,br,bl,Lifter);
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
