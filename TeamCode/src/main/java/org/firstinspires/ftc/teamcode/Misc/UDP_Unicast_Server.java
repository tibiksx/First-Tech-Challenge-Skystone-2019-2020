package org.firstinspires.ftc.teamcode.Misc;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

import java.util.concurrent.Semaphore;


public class UDP_Unicast_Server implements Runnable {
    private final int clientPort;
    public static boolean kill = false;


    public UDP_Unicast_Server(int clientPort) {
        this.clientPort = clientPort;
    }

    private Semaphore sendLock = new Semaphore(1); //used as a lock really

    private long lastSendMillis = 0;

    @Override
    public void run() {
        while (true) {
            if (kill) break;
            try {
                //never send data too fast
                if (SystemClock.uptimeMillis() - lastSendMillis < 50) {
                    continue;
                }
                //set the last send time
                lastSendMillis = SystemClock.uptimeMillis();

                //wait for semaphore to be available
                sendLock.acquire();

                if(currentUpdate.length() > 0){
                    //send the current update
                    SendUPDRaw(currentUpdate);
                    //now we scrap everything in currentUpdate to flag it is empty
                    currentUpdate = "";
                }else{
                    //if we are here, the currentUpdate is empty
                    if(lastUpdate.length() > 0){
                        SendUPDRaw(lastUpdate);
                        //now we scrap everything in lastUpdate to flag it is empty
                        lastUpdate = "";
                    }
                }

                //release the semaphore
                sendLock.release();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void SendUPDRaw(String message)
    {
        try(DatagramSocket serverSocket = new DatagramSocket()) {
            DatagramPacket datagramPacket = new DatagramPacket(message.getBytes(),message.length()
                    ,InetAddress.getByName("192.168.0.1"),clientPort);
            serverSocket.send(datagramPacket);
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (UnknownHostException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String lastUpdate = "";
    public String currentUpdate = "";

    public void addMessage(String string){
        //depending on the state of the semaphore we can do two things
        if(!sendLock.tryAcquire()){
            //if it is being used, set the last update
            lastUpdate = string;
        }else{
            //we can update the current update if we got past the semaphore
            currentUpdate = string;
            //release the semaphore since we have acquired
            sendLock.release();
        }


    }
}
