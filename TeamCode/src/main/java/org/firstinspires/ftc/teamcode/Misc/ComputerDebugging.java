package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Disabled
public class ComputerDebugging {
    public static UDP_Unicast_Server udpServer;
    public Robot rbt = new Robot();

    public StringBuilder messageBuilder = new StringBuilder();
    public ComputerDebugging()
    {
        if(rbt.usingDebugger) {return;}
        udpServer.kill = false;
        udpServer = new UDP_Unicast_Server(50000);
        Thread runner = new Thread(udpServer);
        runner.start(); //have at 'em
    }

    public void stopAll()
    {
        if(rbt.usingDebugger) { return; }
        udpServer.kill = true;

    }

    public  void markEndOfUpdate()
    {
        if(!rbt.usingDebugger) {return; }
        //messageBuilder.append("END");

        udpServer.addMessage(messageBuilder.toString());
        messageBuilder = new StringBuilder();

    }
}
