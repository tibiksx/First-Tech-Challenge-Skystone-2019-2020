package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

@Disabled
public class UDP_Unicast_Server implements Runnable {
    private final int clientPort;

    public UDP_Unicast_Server(int clientPort)
    {
        this.clientPort = clientPort;
    }

    @Override
    public void run() {
        try(DatagramSocket serverSocket = new DatagramSocket(50000))
        {
            String message = "";
            DatagramPacket datagramPacket = new DatagramPacket(message.getBytes(),message.length(),
                    InetAddress.getByName("192.168.43.9"),clientPort);
            serverSocket.send(datagramPacket);

        } catch (UnknownHostException e) {
            e.printStackTrace();
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
