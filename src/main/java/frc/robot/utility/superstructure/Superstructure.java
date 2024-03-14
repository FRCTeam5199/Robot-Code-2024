package frc.robot.utility.superstructure;

import frc.robot.utility.BrakeButton;
import frc.robot.utility.StopClimbButton;

public class Superstructure {
    static StopClimbButton stopClimb = new StopClimbButton(2);
    static BrakeButton brake1 = new BrakeButton(0);
    private static boolean brake;
    private static boolean stop;

    public static void update() {
        stop = stopClimb.get();
        brake = brake1.get();
    }

    public static boolean getClimbButtonPressed() {
        return stop;
    }

    public static boolean getBrakeButtonPressed() {
        return brake;
    }
}