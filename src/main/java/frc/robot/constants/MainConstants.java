package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.PID;


public class MainConstants {

    public static final double ROTATIONS_PER_1_DEGREE_ARM = 0.3825;
    public static final double ARM_PIVOT_X_OFFSET = 0.2032; //meters
    public static final double ARM_PIVOT_Z = 0.5715; //meters
    public static final double ARM_ORIGINAL_DEGREES = 22;
    // public static final double SPEAKER_Z = 2.340102;
    public static final double SPEAKER_Z = 1.4511020000000001 + 0.75;
    public static final double ARM_GEAR_RATIO = 60; //60 : 1
    //Mechanical Constants
    public static String CAN_BUS = "";
    public static double WHEEL_DIAMETER = 4;
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
    public String[] cameraNames = {"Front", "Left", "Right", "Back", "Shooter"};
    public Transform3d[] cameraPositions = {
            new Transform3d(0.2921, 0, -0.123835, new Rotation3d(Math.toRadians(55), 0, Math.toRadians(0))), //front
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)), //left
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)), //right
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0))
    };

    public static class OperatorConstants {
        //Operator Constants
        public static int MAIN_CONTROLLER_PORT = 0;
        public static int OPERATOR_CONTROLLER_PORT = 1;
        public static int TOP_BUTTON_PANEL_PORT = 1;
        public static int BOTTOM_BUTTON_PANEL_PORT = 2;
    }

    public static class IDs {
        // Pigeon
        public static int PIGEON2_ID = 22;

        // Pneumatics
        // public static final int PCM_ID = 0;
        // public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = null;

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

            //Intake
            public static final int INTAKE_ACTUATOR_MOTOR_ID = 9;
            public static final int INTAKE_MOTOR_ID = 10;

            //Climber
            public static final int CLIMBER_MOTOR_1_ID = 7;
            public static final int CLIMBER_MOTOR_2_ID = 8;

            //Arm
            public static final int ARM_MOTOR_ID = 5;
            public static final int ARM_MOTOR2_ID = 6;
            public static final int ARM_ENCODER_MOTOR = 1;

            //Shooter
            public static final int SHOOTER_MOTOR_1_ID = 2;
            public static final int SHOOTER_MOTOR_2_ID = 3;
            public static final int SHOOTER_INDEXER_MOTOR_ID = 4;
        }
    }

    public static class PIDConstants {
        public static final PID INTAKE_PID = new PID(0.1, 0, 0);
        // Climber
        public static final PID CLIMBER_PID = new PID(0.01, 0, 0);

        // Arm
        public static final PID ARM_PID = new PID(0.0075, 0.0, 0);
    }

    public static class Setpoints {
        //Intake Setpoints
        public static final double STOW_INTAKE = 0;
        public static final double DEPLOY_INTAKE = 7.5;
        //24

        public static final double CLIMBER_EXTENDED_SETPOINT = 115;
        public static final double CLIMBER_RETRACTED_SETPOINT = 0;

        //Arm Setpoints
        public static final double ARM_STABLE_SETPOINT = 23; //Maybe make it 22.5 for chain
        public static final double ARM_SPEAKER_FRONT_SETPOINT = 27;
        public static final double ARM_INTAKE_SETPOINT = 2;
        public static final double ARM_TRAP_SETPOINT = 130;
        public static final double ARM_TRAP_PREP_SETPOINT = 82;
        public static final double ARM_TRAP_PREP2_SETPOINT = 72;
        public static final double ARM_TOP_PIECE_SETPOINT = 70;
        public static final double ARM_BOTTOM_PIECE_SETPOINT = 73;
        public static final double ARM_MIDDLE_PIECE_SETPOINT = 73;
        //Control Panel
        public static final double ARM_SUBWOOFER_SETPOINT = 67;
        public static final double ARM_SPEAKER_BACK_SETPOINT = 160;
        public static final double ARM_SAFE_SETPOINT = 53;
        public static final double ARM_AMP_SETPOINT = 60.5;
        public static final double ARM_FAR_SHOT_SETPOINT = 45.5;
        public static final double ARM_HP_STATION_SETPOINT = 150;

    }
}