package frc.robot.subsystems.drivetrain.swerveDrive;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.OdometryThread;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AbstractConstants;

public class SwerveDrive implements Subsystem {
    private SwerveRequest.ApplyChassisSpeeds driveRequest = new SwerveRequest.ApplyChassisSpeeds();
    SwerveDrivetrain swerveDrive;
    SwerveModules frontLeft;
    SwerveModules frontRight;
    SwerveModules backLeft;
    SwerveModules backRight;
    SwerveDriveOdometry odometry;



    public SwerveDrive() {
        SwerveModules frontLeft = new SwerveModules(AbstractConstants.FRONT_LEFT_DRIVE_MOTOR_ID, AbstractConstants.FRONT_LEFT_STEER_MOTOR_ID, AbstractConstants.FRONT_LEFT_ENCODER, AbstractConstants.CAN_BUS, AbstractConstants.FRONT_LEFT_SWERVE_OFFSET, false, false);
        SwerveModules frontRight = new SwerveModules(AbstractConstants.FRONT_RIGHT_DRIVE_MOTOR_ID, AbstractConstants.FRONT_RIGHT_STEER_MOTOR_ID, AbstractConstants.FRONT_RIGHT_ENCODER,AbstractConstants.CAN_BUS, AbstractConstants.FRONT_RIGHT_SWERVE_OFFSET, false, false);
        SwerveModules backLeft = new SwerveModules(AbstractConstants.BACK_LEFT_DRIVE_MOTOR_ID, AbstractConstants.BACK_LEFT_STEER_MOTOR_ID, AbstractConstants.BACK_LEFT_ENCODER, AbstractConstants.CAN_BUS, AbstractConstants.BACK_LEFT_SWERVE_OFFSET, false, false);
        SwerveModules backRight = new SwerveModules(AbstractConstants.BACK_RIGHT_DRIVE_MOTOR_ID, AbstractConstants.BACK_RIGHT_STEER_MOTOR_ID, AbstractConstants.BACK_RIGHT_ENCODER, AbstractConstants.CAN_BUS, AbstractConstants.BACK_RIGHT_SWERVE_OFFSET, false, false);
        SwerveModule modules[] = new SwerveModule[4];
        modules[0] = frontLeft.swerveModule;
        modules[1] = frontRight.swerveModule;
        modules[2] = backLeft.swerveModule;
        modules[3] = backRight.swerveModule;

        SwerveModuleConstants[] moduleConstants = new SwerveModuleConstants[4];
        moduleConstants[0] = frontLeft.swerveConstants;
        moduleConstants[1] = frontRight.swerveConstants;
        moduleConstants[2] = backLeft.swerveConstants;
        moduleConstants[3] = backRight.swerveConstants;

        SwerveDrivetrainConstants driveConstants = new SwerveDrivetrainConstants();

        driveConstants.CANbusName = AbstractConstants.CAN_BUS;
        driveConstants.Pigeon2Id = AbstractConstants.PIGEON2_ID;
        driveConstants.SupportsPro = true;

        swerveDrive = new SwerveDrivetrain(driveConstants, 0, moduleConstants);

        swerveDrive.seedFieldRelative();

    }


    public void drive(ChassisSpeeds sppesds){
        swerveDrive.setControl(driveRequest.withSpeeds(sppesds));
    }

    
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return AbstractConstants.SWERVE_KINEMATICS.toChassisSpeeds(swerveDrive.getState().ModuleStates);
    }

    public ChassisSpeeds getRelativeChassisSpeeds(){
        return ChassisSpeeds.fromFieldRelativeSpeeds(getCurrentRobotChassisSpeeds(), getRotation());
    }

    public Pose2d getPoseVision(){

        return swerveDrive.getState().Pose;
    }

    public Pose2d getPose(){
        return swerveDrive.getState().Pose;
    }
    
    public void resetOdometry(Pose2d pose){
        swerveDrive.seedFieldRelative(pose);
    }

    public Rotation2d getRotation(){
        return swerveDrive.getState().Pose.getRotation();
    }



    
}

