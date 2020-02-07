package org.firstinspires.ftc.teamcode.Threads;

import java.io.File;

public class MusicThread implements Runnable {

    private String soundPath = "/FIRST/blocks/sounds";
    private File goldFile   = new File("/sdcard" + soundPath + "/gold.wav");
    private File silverFile = new File("/sdcard" + soundPath + "/silver.wav");

    @Override
    public void run() {

    }
}
