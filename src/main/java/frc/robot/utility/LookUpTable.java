package frc.robot.utility;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<AutoAimValue> firstList = new ArrayList<>(); //0 - 1 meters
    public static ArrayList<AutoAimValue> secondList = new ArrayList<>(); //1 - 2 meters
    public static ArrayList<AutoAimValue> thirdList = new ArrayList<>(); //2 - 3 meters
    public static ArrayList<AutoAimValue> fourthList = new ArrayList<>(); //3+ meters

    public static AutoAimValue findValue(double distance) {
        ArrayList<AutoAimValue> listUsed;
        if (distance >= 0 && distance <= 1) {
            listUsed = firstList;
        } else if (distance >= 1 && distance <= 2) {
            listUsed = secondList;
        } else if (distance >= 2 && distance <= 3) {
            listUsed = thirdList;
        } else {
            listUsed = fourthList;
        }

        AutoAimValue lowAutoAimValue = listUsed.get(0);
        AutoAimValue highAutoAimValue = listUsed.get(listUsed.size() - 1);
        for (int i = 0; i < listUsed.size() - 1; i++) {
            if (distance > listUsed.get(i).distance && distance < listUsed.get(i + 1).distance) {
                lowAutoAimValue = listUsed.get(i);
                highAutoAimValue = listUsed.get(i + 1);
                break;
            }
        }

        double percentInBetween = (distance - lowAutoAimValue.distance) / (highAutoAimValue.distance - lowAutoAimValue.distance);
        double armAngle = highAutoAimValue.armAngle - (percentInBetween * (highAutoAimValue.armAngle - lowAutoAimValue.armAngle);
        double shooterRPM = lowAutoAimValue.shooterRPM + (percentInBetween * (highAutoAimValue.shooterRPM - lowAutoAimValue.shooterRPM);

        return new AutoAimValue(distance, armAngle, shooterRPM);
    }
}
