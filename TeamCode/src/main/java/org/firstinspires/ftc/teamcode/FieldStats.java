package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Point;

public class FieldStats {

    public static class ParkingPlaces {
        public static Point RED_INWARD_PARKING = new Point(266.2,360/2.0);
        public static Point BLUE_INWARD_PARKING = new Point(93.3,360/2.0);

        public static Point RED_OUTWARD_PARKING = new Point(334.8,360/2.0);
        public static Point BLUE_OUTWARD_PARKING = new Point(25.5,360/2.0);
    }

    public static class REDFoundation {
        public static Point DEFAULT_placingPosition = new Point(260,305.0);
        public static Point AFTER_AUTO_placingPosition = new Point(292.02,288.3);
    }

    public static class BLUEFoundation {
        public static Point DEFAULT_placingPosition = new Point(97.5,305.0);
        public static Point AFTER_AUTO_placingPosition = new Point(306.0,288.3);
    }


    public static class REDStones {
        public static Stone nullStone = new Stone(new Point(0,0),false);
        public static Stone firstStone = new Stone(new Point(231.24+30,10.16),false);
        public static Stone secondStone = new Stone(new Point(231.24+30,10.16 + 20.32),false);
        public static Stone thirdStone = new Stone(new Point(231.24+30,10.16 + 20.32 + 20.32),false);
        public static Stone fourthStone = new Stone(new Point(231.24+30,10.16 + 20.32 + 20.32 + 20.32),false);
        public static Stone fifthStone = new Stone(new Point(231.24+30,10.16 + 20.32 + 20.32 + 20.32 + 20.32),false);
        public static Stone sixthStone = new Stone(new Point(231.24+30,10.16 + 20.32 + 20.32 + 20.32 + 20.32 + 20.32),false);

        public static Stone[] stoneArray = {nullStone, firstStone, secondStone, thirdStone, fourthStone, fifthStone, sixthStone};

        public static void markAsSkystone(int stoneIndex) {
            stoneArray[stoneIndex].isSkystone = true;
        }

        public static void displayPattern(Telemetry telemetry) {
            StringBuilder pattern = new StringBuilder();
            for(int i = 1; i<=6; i++) {
                pattern.append(stoneArray[i].isSkystone()).append(" ");
            }

            telemetry.addData("Pattern: ", pattern.toString());
            telemetry.update();
        }
    }

    public static class BLUEStones {
        public static Stone nullStone = new Stone(new Point(0,0),false);
        public static Stone firstStone = new Stone(new Point(122.2,10.16),false);
        public static Stone secondStone = new Stone(new Point(122.2,10.16 + 20.32),false);
        public static Stone thirdStone = new Stone(new Point(122.2,10.16 + 20.32 + 20.32),false);
        public static Stone fourthStone = new Stone(new Point(122.2,10.16 + 20.32 + 20.32 + 20.32),false);
        public static Stone fifthStone = new Stone(new Point(122.2,10.16 + 20.32 + 20.32 + 20.32 + 20.32),false);
        public static Stone sixthStone = new Stone(new Point(122.2,10.16 + 20.32 + 20.32 + 20.32 + 20.32 + 20.32),false);

        public static Stone[] stoneArray = {nullStone, firstStone, secondStone, thirdStone, fourthStone, fifthStone, sixthStone};

        public static void markAsSkystone(int stoneIndex) {
            stoneArray[stoneIndex].isSkystone = true;
        }

        public static void displayPattern(Telemetry telemetry) {
            StringBuilder pattern = new StringBuilder();
            for(int i = 1; i<=6; i++) {
                pattern.append(stoneArray[i].isSkystone()).append(" ");
            }

            telemetry.addData("Pattern: ", pattern.toString());
            telemetry.update();
        }
    }

    public static class Stone {
        private Point position;
        private boolean isSkystone;

        public Stone(Point position, boolean isSkystone) {
            this.position = position;
            this.isSkystone = isSkystone;
        }

        public boolean isSkystone() {return isSkystone;}

        public Point getPosition() {
            return position;
        }
    }
}
