package frc.robot.Constants;
import frc.robot.Main;
import frc.robot.Constants.AbstractConstants;

public class MainConstants extends AbstractConstants {
    public MainConstants(){
//ID's
    DRIVE_MOTOR_ID = 1;
    STEER_MOTOR_ID = 2;
    CANCODER_ID = 0;
    PIGEON2_ID = 0;

//Mechanical Constants
    CAN_BUS = "rio";
    WHEEL_DIAMETER = 0;
    DRIVE_GEAR_RATIO = 0;
    STEER_GEAR_RATIO = 0;
    //All swerve offsets are in degrees.
    FRONT_LEFT_SWERVE_OFFSET = 0;
    FRONT_RIGHT_SWERVE_OFFSET = 0;
    BACK_LEFT_SWERVE_OFFSET = 0;
    BACK_RIGHT_SWERVE_OFFSET = 0;
  
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

