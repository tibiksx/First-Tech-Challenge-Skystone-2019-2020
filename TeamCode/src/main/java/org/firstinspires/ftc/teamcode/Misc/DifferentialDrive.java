package org.firstinspires.ftc.teamcode.Misc;

public class DifferentialDrive {

    private static final double distantBetweenWheels = 40.5;   //in cm
   public static double getDifferentialPower(double radius)
   {
        return ( (radius - distantBetweenWheels/2) / (radius + distantBetweenWheels) );
   }
}
