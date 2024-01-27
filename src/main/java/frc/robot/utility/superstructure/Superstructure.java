package frc.robot.utility.superstructure;

import frc.robot.utility.BrakeButton;

public class Superstructure {
    private static boolean brake;

    public static void update() {
        brake = BrakeButton.get();
    }

    public static boolean getBrakeButtonPressed() {
        return brake;
    }
}
