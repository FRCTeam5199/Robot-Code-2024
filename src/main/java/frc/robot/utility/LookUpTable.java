package frc.robot.utility;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<AutoAimValue> listOfValues = new ArrayList<>() {
        {
            //+20 top    /    -15 bottom
            add(new AutoAimValue(1.36, 82.3, 3800));
            add(new AutoAimValue(1.96, 68.1, 4200));
            add(new AutoAimValue(2.22, 67.2, 4250));
            add(new AutoAimValue(2.32, 64, 4300));
            add(new AutoAimValue(2.66, 61, 4800));
            add(new AutoAimValue(3.08, 58.2, 5300));
            add(new AutoAimValue(3.23, 56.8, 5400));
            add(new AutoAimValue(3.6, 55.3, 5500));
            add(new AutoAimValue(4, 54.8, 5600));
            add(new AutoAimValue(4.48, 51.8, 5750));
            add(new AutoAimValue(4.73, 51.2, 5800));
            add(new AutoAimValue(5.11, 50.8, 5900));
            add(new AutoAimValue(5.17, 49.5, 6000));
            add(new AutoAimValue(5.46, 48.8, 6000));
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
