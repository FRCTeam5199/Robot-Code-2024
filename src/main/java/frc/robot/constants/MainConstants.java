package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import frc.robot.Main;
import frc.robot.constants.AbstractConstants;


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
    CAN_BUS = "Canivore1";
    WHEEL_DIAMETER = 4;
    DRIVE_GEAR_RATIO = 6.75;
    STEER_GEAR_RATIO = 12.8;
    //All swerve offsets are in degrees.
    FRONT_LEFT_SWERVE_OFFSET = 301;
    FRONT_RIGHT_SWERVE_OFFSET = 304;
    BACK_LEFT_SWERVE_OFFSET = 269.5;
    BACK_RIGHT_SWERVE_OFFSET = 157.5;
  
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


    String[] cameraNames = {"1","2","3"};
    FRONT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    FRONT_CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    TARGET_HEIGHT_METERs = 0;


    // final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  
    GOAL_RANGE_METERS = Units.feetToMeters(2);

    ENABLE_OVERHEAT_DETECTION = false;
    OVERHEAT_THRESHOLD = 80;
    CTRE_SENSOR_UNITS_PER_ROTATION = 2048;
    }

    
}

// package frc.robot.constants;

// public class MainConstants extends AbstractConstants {
//     public MainConstants(){
// //ID's
//     FRONT_LEFT_DRIVE_MOTOR_ID = 1;
//     FRONT_LEFT_STEER_MOTOR_ID = 2;
//     FRONT_LEFT_ENCODER = 11;

//     FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
//     FRONT_RIGHT_STEER_MOTOR_ID = 4;
//     FRONT_RIGHT_ENCODER = 12;

//     BACK_LEFT_DRIVE_MOTOR_ID = 5;
//     BACK_LEFT_STEER_MOTOR_ID = 6;
//     BACK_LEFT_ENCODER = 13;

//     BACK_RIGHT_DRIVE_MOTOR_ID = 7;
//     BACK_RIGHT_STEER_MOTOR_ID = 8;
//     BACK_RIGHT_ENCODER = 14;

//     PIGEON2_ID = 22;

// //Mechanical Constants
//     CAN_BUS = "rio";
//     WHEEL_DIAMETER = 4;
//     DRIVE_GEAR_RATIO = 6.75;
//     STEER_GEAR_RATIO = 12.8;
//     //All swerve offsets are in degrees.
//     FRONT_LEFT_SWERVE_OFFSET = 21.21;
//     FRONT_RIGHT_SWERVE_OFFSET = 206.2;
//     BACK_LEFT_SWERVE_OFFSET = 170.2;
//     BACK_RIGHT_SWERVE_OFFSET = 248.007;
  
// //Drive Constants
//     INVERT_FL_DRIVE = false;
//     INVERT_FL_STEER = false;

//     INVERT_FR_DRIVE = false;
//     INVERT_FR_STEER = false;

//     INVERT_BL_DRIVE = false;
//     INVERT_BL_STEER = false;

//     INVERT_BR_DRIVE = false;
//     INVERT_BR_STEER = false;
    

//     /**
//      * d
//      */
    

//     }

// }

