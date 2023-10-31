package frc.robot.Constants;
import frc.robot.Main;
import frc.robot.Constants.AbstractConstants;

public class MainConstants extends AbstractConstants {
    public MainConstants(){
//ID's
    FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    FRONT_LEFT_STEER_MOTOR_ID = 2;
    FRONT_LEFT_ENCODER = 11;

    FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
    FRONT_RIGHT_STEER_MOTOR_ID = 4;
    FRONT_RIGHT_ENCODER = 12;

    BACK_LEFT_DRIVE_MOTOR_ID = 5;
    BACK_LEFT_STEER_MOTOR_ID = 6;
    BACK_LEFT_ENCODER = 13;

    BACK_RIGHT_DRIVE_MOTOR_ID = 7;
    BACK_RIGHT_STEER_MOTOR_ID = 8;
    BACK_RIGHT_ENCODER = 14;

    PIGEON2_ID = 22;

//Mechanical Constants
    CAN_BUS = "rio";
    WHEEL_DIAMETER = 4;
    DRIVE_GEAR_RATIO = 6.75;
    STEER_GEAR_RATIO = 12.8;
    //All swerve offsets are in degrees.
    FRONT_LEFT_SWERVE_OFFSET = 21.21;
    FRONT_RIGHT_SWERVE_OFFSET = 206.2;
    BACK_LEFT_SWERVE_OFFSET = 170.2;
    BACK_RIGHT_SWERVE_OFFSET = 248.007;
  
//Drive Constants
    INVERT_FL_DRIVE = false;
    INVERT_FL_STEER = false;

    INVERT_FR_DRIVE = false;
    INVERT_FR_STEER = false;

    INVERT_BL_DRIVE = false;
    INVERT_BL_STEER = false;

    INVERT_BR_DRIVE = false;
    INVERT_BR_STEER = false;
    

//Operator Constants
    int CONTROLLER_PORT = 0;
    int BUTTON_PANEL_PORT = 0;
    }

}

