package org.firstinspires.ftc.teamcode.FieldStats;

import org.firstinspires.ftc.teamcode.Misc.Point;

public class StonesStats {
    public static Stone firstStone;
    public static Stone secondStone;
    public static Stone thirdStone;
    public static Stone fourthStone;
    public static Stone fifthStone;

    public static void initStones() {
        firstStone = new Stone(new Point(0,0),false);
        secondStone = new Stone(new Point(0,0),false);
        thirdStone = new Stone(new Point(0,0),false);
        fourthStone = new Stone(new Point(0,0),false);
        fifthStone = new Stone(new Point(0,0),false);
    }




}
