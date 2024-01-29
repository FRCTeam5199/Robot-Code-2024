package frc.robot.utility.superstructure;

import frc.robot.utility.BrakeButton;

public class Superstructure {
    static BrakeButton brake1 = new BrakeButton(0);
    private static boolean brake;

    public static void update() {
        brake = brake1.get();
    }

    public static boolean getBrakeButtonPressed() {
        return brake;
    }
}
