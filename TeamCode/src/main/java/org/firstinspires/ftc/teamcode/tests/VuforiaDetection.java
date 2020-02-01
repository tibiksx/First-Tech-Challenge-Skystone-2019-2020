package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware;

public class VuforiaDetection extends LinearOpMode {

    private Hardware robot = new Hardware();

    private static final String VUFORIA_KEY =
            "ATcd+rr/////AAABmYgBcljNLEdpiqJCIVV/ViQClFkFihZXAgumX6I+WuvyzdqkybzXA48bUHj6UcIUG+/Sgb432D/yxlEyvpCo1pWJ1c37D1iGTpForaTtdLkRPq/Yc3AfY0NYseJulDZEaCBYvyx7ruL1UbDBRwktkpz9ru0XXNIlCPBGF1l9i8aOzI+xxF39ol+MpxXZuvjVPUluovzHn735bT8ua9BPIyFVjaYrkUhzjiEwYyxnTZGIQiRRrWtH97tM9xQe2GndA5piYZU51D3nqfrIPVcSrUgtxfTFPaLGMMQu2wu9RhM0dKpKe0NMqxCCQu8ybSWqwcIrAyXNaSX7sbPY1LKA8CExKhSBOlA3p1XUJA6wdPEt";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);


    }
}
