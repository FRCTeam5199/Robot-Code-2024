package frc.robot.utility;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<AutoAimValue> listOfValues = new ArrayList<>() {
        {
            add(new AutoAimValue(1.36, 82.3, 4000));
            add(new AutoAimValue(1.96, 68.1, 4500));
            add(new AutoAimValue(2.77, 61.8, 5000));
            //+20 top    /    -15 bottom
            add(new AutoAimValue(3.62, 52.1, 5500));
        }
    };

    public static AutoAimValue findValue(double distance) {
        // System.out.println("Distance: " + distance);
        AutoAimValue lowAutoAimValue = listOfValues.get(0);
        AutoAimValue highAutoAimValue = listOfValues.get(listOfValues.size() - 1);
        for (int i = 0; i < listOfValues.size() - 1; i++) {
            if (distance > listOfValues.get(i).distance && distance < listOfValues.get(i + 1).distance) {
                lowAutoAimValue = listOfValues.get(i);
                highAutoAimValue = listOfValues.get(i + 1);
                break;
            }
        }

        // System.out.println("Low Auto Aim Value: " + lowAutoAimValue.distance);
        // System.out.println("High Auto Aim Value: " + highAutoAimValue.distance);

        double percentInBetween = (distance - lowAutoAimValue.distance) / (highAutoAimValue.distance - lowAutoAimValue.distance);
        // System.out.println("Percent: " + percentInBetween);
        double armAngle = lowAutoAimValue.armAngle - (percentInBetween * (lowAutoAimValue.armAngle - highAutoAimValue.armAngle));
        // System.out.println("Offset added: " + (percentInBetween * (lowAutoAimValue.armAngle - highAutoAimValue.armAngle)));
        // System.out.println("High Auto Aim Value: " + highAutoAimValue.armAngle);
        // System.out.println("Arm angle: " + armAngle);

        double shooterRPM = lowAutoAimValue.shooterRPM + (percentInBetween * (highAutoAimValue.shooterRPM - lowAutoAimValue.shooterRPM));

        return new AutoAimValue(distance, armAngle, shooterRPM);
    }
}
