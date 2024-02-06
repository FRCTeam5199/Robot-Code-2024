package frc.robot.utility;

import frc.robot.utility.StopClimbButton;

public class Superstructure {
    static StopClimbButton stopClimb = new StopClimbButton(2);
    private static boolean stop;

    public static void update() {
        stop = stopClimb.get();
    }

    public static boolean getClimbButtonPressed() {
        return stop;
    }
}