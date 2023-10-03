// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/**
 * Constants that are applicable to every bot.
 */
public abstract class AbstractConstants {

    public static int DRIVE_MOTOR_ID = 0;
    public static int STEER_MOTOR_ID = 0;
    public static int CANCODER_ID = 0;
    public static int PIGEON2_ID = 0;

    public static String CAN_BUS = "rio";
    public static double WHEEL_DIAMETER = 0;
    public static double DRIVE_GEAR_RATIO = 0;
    public static double STEER_GEAR_RATIO = 0;
    
    //All swerve offsets are in degrees.
    public static double FRONT_LEFT_SWERVE_OFFSET = 0;
    public static double FRONT_RIGHT_SWERVE_OFFSET = 0;
    public static double BACK_LEFT_SWERVE_OFFSET = 0;
    public static double BACK_RIGHT_SWERVE_OFFSET = 0;


    public static boolean INVERT_FL_DRIVE = false;
    public static boolean INVERT_FL_STEER = false;

    public static boolean INVERT_FR_DRIVE = false;
    public static boolean INVERT_FR_STEER = false;

    public static boolean INVERT_BL_DRIVE = false;
    public static boolean INVERT_BL_STEER = false;

    public static boolean INVERT_BR_DRIVE = false;
    public static boolean INVERT_BR_STEER = false;
    

  

    public static int CONTROLLER_PORT = 0;
    public static int BUTTON_PANEL_PORT = 0;


}
