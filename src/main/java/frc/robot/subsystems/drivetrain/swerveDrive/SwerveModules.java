package frc.robot.subsystems.drivetrain.swerveDrive;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;


public class SwerveModules {
    SwerveModuleConstants swerveConstants;
    FeedbackConfigs feedback;
    SwerveModule swerveModule;

    public SwerveModules(int DriveMotorID, int SteerMotorID, int CanCoderID, String canbus, double CANcoderOffset, boolean InvertDrive, boolean InvertSteer){
        swerveConstants.DriveMotorId = DriveMotorID;
        swerveConstants.CANcoderId = CanCoderID;
        swerveConstants.SteerMotorId = SteerMotorID;
        swerveConstants.CANcoderOffset = CANcoderOffset;

        swerveConstants.DriveMotorGearRatio = 6.12;
        swerveConstants.SteerMotorGearRatio = 12.8;
        swerveConstants.WheelRadius = 4; //inches

        swerveConstants.DriveMotorInverted = InvertDrive;
        swerveConstants.SteerMotorInverted = InvertSteer;

        swerveModule = new SwerveModule(swerveConstants, canbus, true);
    }

    
    
}
