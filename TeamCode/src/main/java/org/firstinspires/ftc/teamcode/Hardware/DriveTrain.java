package org.firstinspires.ftc.teamcode.Hardware;
import android.os.SystemClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.revextensions2.ExpansionHubMotor;
public class DriveTrain {


    public DriveTrain(ExpansionHubMotor fr, ExpansionHubMotor fl, ExpansionHubMotor br, ExpansionHubMotor bl, ExpansionHubMotor Lifter)
    {
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
}
