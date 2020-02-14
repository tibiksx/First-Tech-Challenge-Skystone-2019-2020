package org.firstinspires.ftc.teamcode.FieldStats;

import org.firstinspires.ftc.teamcode.Misc.Point;

public class Stone {
    public Point position;
    public boolean isSkystone = false;

    Stone(Point position, boolean isSkystone)
    {
        this.position = position;
        this.isSkystone = isSkystone;
    }
}
