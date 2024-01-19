
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.MainConstants;

import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalDouble;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class AprilTagSubsystem implements Subsystem {

    public MainConstants Constants = new MainConstants();
    EstimatedRobotPose[] robotPose = new EstimatedRobotPose[4];
    SwerveDrive drive = TunerConstants.DriveTrain;



    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = limelight.getEntry("tx");
    NetworkTableEntry ty = limelight.getEntry("ty");
    NetworkTableEntry tz = limelight.getEntry("tz");
    double x = tx.getDouble(0);
    double y = ty.getDouble(0);
    double z = tz.getDouble(0);


    AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    PhotonPoseEstimator.PoseStrategy poseStrategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    PhotonPoseEstimator multiPoseEstimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Back"), Constants.cameraPositions[3]);
    PhotonPoseEstimator singlePoseEstimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Back"), Constants.cameraPositions[3]);

    public static PhotonCamera frontCamera;
    public static PhotonCamera leftCamera;
    public static PhotonCamera rightCamera;
    public static PhotonCamera backCamera;
    // 0 Front, 1 Back, 2 Left, 3 Rigggggggggggggggght
    public PhotonCamera[] allCameras = {frontCamera, rightCamera, leftCamera, backCamera};
    public PhotonTrackedTarget[] bestTargetFromCameras;
    public MultiTargetPNPResult[] multiTargetPNPResults;


    // public static PhotonCamera[] cameraDirections = {front, left, rigth, back};

    public AprilTagSubsystem() {
        allCameras[3] = new PhotonCamera("Back");
    }




    public String getAllianceColor() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                return "Red";
            }
            if (ally.get() == Alliance.Blue) {
                return "Blue";
            }
        }
        return null;

    }


    public Optional<EstimatedRobotPose> getVisionPoseFront() {
        var result = allCameras[0].getLatestResult();
        if(result.getMultiTagResult().estimatedPose.isPresent && result.getMultiTagResult().estimatedPose.ambiguity < .2){
            return multiPoseEstimator.update();
        }else if(result.hasTargets()){
            return singlePoseEstimator.update();

        }else{
            System.out.println("O Tags on Front");
            return Optional.empty();
        }
    }
    public Optional<EstimatedRobotPose> getVisionPoseRight() {
        var result = allCameras[1].getLatestResult();
        if(result.getMultiTagResult().estimatedPose.isPresent && result.getMultiTagResult().estimatedPose.ambiguity < .2){
            return multiPoseEstimator.update();
        }else if(result.hasTargets()){
            return singlePoseEstimator.update();

        }else{
            System.out.println("O Tags On Right");
            return Optional.empty();
        }
    }
    public Optional<EstimatedRobotPose> getVisionPoseLeft() {
        var result = allCameras[2].getLatestResult();
        if(result.getMultiTagResult().estimatedPose.isPresent && result.getMultiTagResult().estimatedPose.ambiguity < .2){
            return multiPoseEstimator.update();
        }else if(result.hasTargets()){
            return singlePoseEstimator.update();

        }else{
            System.out.println("O Tags On Left");
            return Optional.empty();
        }
    }
    public Optional<EstimatedRobotPose> getVisionPoseBack() {
        var result = allCameras[3].getLatestResult();
        if(result.getMultiTagResult().estimatedPose.isPresent && result.getMultiTagResult().estimatedPose.ambiguity < .2){
            return multiPoseEstimator.update();
        }else if(result.hasTargets()){
            return singlePoseEstimator.update();

        }else{
            System.out.println("O Tags Back");
            return Optional.empty();
        }
    }

}
