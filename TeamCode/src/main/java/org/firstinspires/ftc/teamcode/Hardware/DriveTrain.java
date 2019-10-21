package org.firstinspires.ftc.teamcode.Hardware;
import android.os.SystemClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.revextensions2.ExpansionHubMotor;
public class DriveTrain {

    public ExpansionHubMotor fr;
    public ExpansionHubMotor fl;
    public ExpansionHubMotor br;
    public ExpansionHubMotor bl;
    public DriveTrain()
    {
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
