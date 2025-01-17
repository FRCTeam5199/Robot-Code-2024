package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.PID;

public class MainConstants {
    public static final double ROTATIONS_PER_1_DEGREE_ARM = 0.3825;
    public static final double ARM_PIVOT_X_OFFSET = 0.18415; //0.2032; //meters //7.25 inch
    public static final double ARM_PIVOT_Z = 0.5800;//0.5788; //0.5715; //meters //22 inch
    public static final double ARM_ORIGINAL_DEGREES = 29.5;
    // public static final double SPEAKER_Z = 2.340102;
    public static final double SPEAKER_Z = 1.4511020000000001 + 0.678; //+ 0.755; //1.4511020000000001 + 0.635;
    public static final double ARM_GEAR_RATIO = 60; //60 : 1
    public static final int LED_LENGTH = 34;
    //Mechanical Constants
    public static String CAN_BUS = "";
    public static double WHEEL_DIAMETER = 2;
    public static double DRIVE_GEAR_RATIO = 5.36;
    public static double STEER_GEAR_RATIO = 12.8;
    //All swerve offsets are in degrees.
    public static double DRIVETRAIN_TRACKWIDTH_METERS = .4953;
    public static double DRIVETRAIN_WHEELBASE_METERS = .6477;
    public static SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    public static int krakenShooter = 0;
    //Drive Constants
    public static boolean INVERT_FL_DRIVE = false;
    public static boolean INVERT_FL_STEER = false;
    public static boolean INVERT_FR_DRIVE = false;
    public static boolean INVERT_FR_STEER = false;
    public static boolean INVERT_BL_DRIVE = false;
    public static boolean INVERT_BL_STEER = false;
    public static boolean INVERT_BR_DRIVE = false;
    public static boolean INVERT_BR_STEER = false;
    public static double MAX_VELOCITY_METERS_PER_SECOND = 0;
    //Operator Constants
    public static int CONTROLLER_PORT = 0;
    public static int BUTTON_PANEL_PORT = 0;
    //right hp red 9, left hp 10 // right hp blue 1, left hp blue 2
    //middle red speaker 4, right red speaker 3 // middle blue speaker 7, left blue speaker 8
    //red amp 5 // blue amp 6
    //red stage facing enemy 13, other 11,12 // blue stage facing enemy 14, other 15 ,16
    public static int rightHPRedID = 9;
    public static int leftHPRedID = 10;
    public static int rightHPBlueID = 1;
    public static int leftHPBlueID = 2;
    public static int middleRedSpeakerID = 4;
    public static int rightRedSpeakerID = 3;
    public static int middleBlueSpeakerID = 7;
    public static int leftBlueSpeakerID = 8;
    public static int redAmpID = 5;
    public static int blueAmpID = 6;
    public static int redstageFront = 13;
    public static int redstageLeft = 11;
    public static int redStageRight = 12;
    public static int bluestageFront = 14;
    public static int bluestageLeft = 15;
    public static int blueStageRight = 16;
    public static double FRONT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    public static double FRONT_CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    // add how invisible setpoint to the target height
    public static double TARGET_HEIGHT_METERS = Units.inchesToMeters(10.5);
    public static double GOAL_RANGE_METERS = Units.feetToMeters(2);
    // final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    public static boolean ENABLE_OVERHEAT_DETECTION = false;
    public static int OVERHEAT_THRESHOLD = 80;
    public static double CTRE_SENSOR_UNITS_PER_ROTATION = 2048;
    public static int BrakeButtonid = 0;
    public static Transform3d[] cameraPositions = {
            new Transform3d(0.4, -.6, 0.123835, new Rotation3d(Math.toRadians(65), 0, Math.toRadians(-10))), //front
            new Transform3d(.225, .28, .0625, new Rotation3d(0, Math.toRadians(11), Math.toRadians(90))), //left
            new Transform3d(.1, -.4, .0625, new Rotation3d(0, Math.toRadians(11), Math.toRadians(-90))), //right
            new Transform3d(-.175, -.4, 0.409, new Rotation3d(Math.toRadians(125), 0, Math.toRadians(180))), //back
            new Transform3d(0, 0
                    , 0, new Rotation3d(0, 0, 0))
    };
    public String[] cameraNames = {"Front", "Back"};

    public static class OperatorConstants {
        //Operator Constants
        public static int MAIN_CONTROLLER_PORT = 0;
        public static int OPERATOR_CONTROLLER_PORT = 1;
        public static int TOP_BUTTON_PANEL_PORT = 1;
        public static int BOTTOM_BUTTON_PANEL_PORT = 2;
    }

    public static class IDs {
        // Pigeon
        public static final int PIGEON2_ID = 22;
        public static final int SERVO_ID = 2;
        public static final int LED_PORT = 0;

        public static class Motors {
            // Drive
            public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 0;
            public static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
            public static final int FRONT_LEFT_ENCODER = 8;

            public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 7;
            public static final int FRONT_RIGHT_STEER_MOTOR_ID = 4;
            public static final int FRONT_RIGHT_ENCODER = 2;

            public static final int BACK_LEFT_DRIVE_MOTOR_ID = 9;
            public static final int BACK_LEFT_STEER_MOTOR_ID = 6;
            public static final int BACK_LEFT_ENCODER = 10;

            public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 5;
            public static final int BACK_RIGHT_STEER_MOTOR_ID = 1;
            public static final int BACK_RIGHT_ENCODER = 13;

            public static final int ARM_ENCODER = 50;

            //Intake
            public static final int INTAKE_ACTUATOR_MOTOR_ID = 30;
            public static final int INTAKE_MOTOR_ID = 32;

            //Climber
            public static final int CLIMBER_MOTOR_1_ID = 34;
            public static final int CLIMBER_MOTOR_2_ID = 33;

            //Arm
            public static final int ARM_MOTOR_LEFT_ID = 38;
            public static final int ARM_MOTOR_RIGHT_ID = 39;

            //Shooter
            public static final int SHOOTER_MOTOR_LEFT_ID = 36;
            public static final int SHOOTER_MOTOR_RIGHT_ID = 37;
            public static final int SHOOTER_INDEXER_MOTOR_ID = 31;
        }
    }

    public static class PIDConstants {
        // Intake PID
        public static final PID INTAKE_PID = new PID(0.085, 0, 0); // 0.07, 0.08, 0.1

        // Climber PID
        public static final PID CLIMBER_PID = new PID(0.01, 0, 0);

        // Arm PID
        public static final PID ARM_PID = new PID(0.0075, 0.0, 0);
    }

    public static class Setpoints {
        // Servo Setpoints
        public static final double SERVO_EXTENDED_SETPOINT = 82;
        public static final double SERVO_RETRACTED_SETPOINT = 20;

        // Intake Setpoints
        public static final double STOW_INTAKE_SETPOINT = 0.5; //0,0.2,0.5
        public static final double DEPLOY_INTAKE_SETPOINT = 8.1; //8.1, 8.2, 8.36, 8.8

        // Climber Setpoints
        public static final double CLIMBER_EXTENDED_SETPOINT = 115;
        public static final double CLIMBER_RETRACTED_SETPOINT = 0;

        // Arm Setpoints
        public static final double ARM_STABLE_SETPOINT = 31; //Maybe make it 22.5 for chain
        public static final double ARM_INTAKE_SETPOINT = 4;
        public static final double ARM_TRAP_SETPOINT = 118.75;
        public static final double ARM_TRAP_PREP_SETPOINT = 84;
        public static final double ARM_TRAP_PREP2_SETPOINT = 72;
        public static final double ARM_TOP_PIECE_SETPOINT = 70;
        public static final double ARM_BOTTOM_PIECE_SETPOINT = 73;
        public static final double ARM_MIDDLE_PIECE_SETPOINT = 73;
        public static final double ARM_AUTON_STABLE_SETPOINT = 147;
        public static final double ARM_TWO_PIECE_EXTENDED_SHOT_RED_SETPOINT = 52.3;
        public static final double ARM_TWO_PIECE_EXTENDED_SHOT_BLUE_SETPOINT = 53.9;
        //Control Panel
        public static final double ARM_SUBWOOFER_SETPOINT = 90; //81
        public static final double ARM_SPEAKER_BACK_SETPOINT = 146.75;
        public static final double ARM_SAFE_SETPOINT = 57.5;
        public static final double ARM_AMP_SETPOINT = 52.5; // 51.5
        public static final double ARM_FAR_SHOT_SETPOINT = 48.5;
        public static final double ARM_HP_STATION_SETPOINT = 140;
        public static final double ARM_SHUTTLE_SETPOINT = 82; //76.75
    }
}