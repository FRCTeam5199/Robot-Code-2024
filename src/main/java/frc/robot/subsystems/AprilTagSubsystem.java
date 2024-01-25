
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import org.photonvision.targeting.PhotonPipelineResult;
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
    PhotonPoseEstimator multiPoseEstimatorFront = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Front"), Constants.cameraPositions[0]);
    PhotonPoseEstimator singlePoseEstimatorFront = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Front"), Constants.cameraPositions[0]);

    PhotonPoseEstimator multiPoseEstimatorRight = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Right"), Constants.cameraPositions[1]);
    PhotonPoseEstimator singlePoseEstimatorRight = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Right"), Constants.cameraPositions[1]);

    PhotonPoseEstimator multiPoseEstimatorLeft = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Left"), Constants.cameraPositions[3]);
    PhotonPoseEstimator singlePoseEstimatorLeft = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Left"), Constants.cameraPositions[3]);

    PhotonPoseEstimator multiPoseEstimatorBack = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Back"), Constants.cameraPositions[3]);
    PhotonPoseEstimator singlePoseEstimatorBack = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Back"), Constants.cameraPositions[3]);

    PhotonPoseEstimator multiPoseEstimatorShooter = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Shooter"), Constants.cameraPositions[4]);
    PhotonPoseEstimator singlePoseEstimatorShooter = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Shooter"), Constants.cameraPositions[4]);

    public static PhotonCamera frontCamera;
    public static PhotonCamera leftCamera;
    public static PhotonCamera rightCamera;
    public static PhotonCamera backCamera;
    public static PhotonCamera shooter;

    // 0 Front, 1 Back, 2 Left, 3 Rigggggggggggggggght
    public PhotonCamera[] allCameras = {frontCamera, rightCamera, leftCamera, backCamera, shooter};
    public PhotonTrackedTarget[] bestTargetFromCameras;
    public MultiTargetPNPResult[] multiTargetPNPResults;


    // public static PhotonCamera[] cameraDirections = {front, left, rigth, back};

    public AprilTagSubsystem() {
        allCameras[0] = new PhotonCamera("Front");

        allCameras[3] = new PhotonCamera("Back");
        allCameras[4] = new PhotonCamera("Shooter");
    }




    /**
     * get alliance
     * @return "Red" or "Blue"
     */
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

    /**
     * estimated posistion from front
     */

    public Optional<EstimatedRobotPose> getVisionPoseFront() {
        var result = allCameras[0].getLatestResult();
        if(result.getMultiTagResult().estimatedPose.isPresent && result.getMultiTagResult().estimatedPose.ambiguity < .2){
            System.out.println("2 Tags Front");

            return multiPoseEstimatorFront.update();
        }else if(result.hasTargets()){
            System.out.println("1 Tag Front");
            return singlePoseEstimatorFront.update();

        }else{
            System.out.println("O Tags on Front");
            return Optional.empty();
        }
    }
    /**
     * estimated pose right 
     */
    public Optional<EstimatedRobotPose> getVisionPoseRight() {
        var result = allCameras[1].getLatestResult();
        if(result.getMultiTagResult().estimatedPose.isPresent && result.getMultiTagResult().estimatedPose.ambiguity < .2){
            return multiPoseEstimatorRight.update();
        }else if(result.hasTargets()){
            return singlePoseEstimatorRight.update();

        }else{
            System.out.println("O Tags On Right");
            return Optional.empty();
        }
    }
    /**
     * estimated pose left
     */
    public Optional<EstimatedRobotPose> getVisionPoseLeft() {
        var result = allCameras[2].getLatestResult();
        if(result.getMultiTagResult().estimatedPose.isPresent && result.getMultiTagResult().estimatedPose.ambiguity < .2){
            return multiPoseEstimatorLeft.update();
        }else if(result.hasTargets()){
            return singlePoseEstimatorLeft.update();

        }else{
            System.out.println("O Tags On Left");
            return Optional.empty();
        }
    }
    /**
     * estimated pose back
     */
    public Optional<EstimatedRobotPose> getVisionPoseBack() {
        var result = allCameras[3].getLatestResult();
        if(result.getMultiTagResult().estimatedPose.isPresent && result.getMultiTagResult().estimatedPose.ambiguity < .2){
            System.out.println("2 Tags Back");
            return multiPoseEstimatorBack.update();
        }else if(result.hasTargets()){
            System.out.println("1 Tag Back");
            return singlePoseEstimatorBack.update();
        }else{
            System.out.println("O Tags Back");
            return Optional.empty();
        }
    }
    /**
     * shooter gamera pose estimation
     */
    public Optional<Pose3d> getShooterVision(){
       PhotonPipelineResult result = allCameras[4].getLatestResult();
       if(singlePoseEstimatorShooter.update().isPresent()){
           return Optional.ofNullable(singlePoseEstimatorShooter.update().get().estimatedPose);
       }else{
           System.out.println("No apriltag visible, Cannot aim");
           return Optional.empty();
       }
    }
    /**
     *gets speed shooter needs to be based on alliane
     *@returns velocity of shooter 
     */
    public double getShooterSpeed(){
        double distanceFromAprilTag = 0;
        if (getAllianceColor().equals("Blue")){
            distanceFromAprilTag = drive.getPose().getTranslation().getDistance(new Translation2d(-0.0380, 5.5478679999999999));
        }
        else if (getAllianceColor().equals("Red")){
            distanceFromAprilTag = drive.getPose().getTranslation().getDistance(new Translation2d(16.579342, 5.5478679999999999));
    
        }
        double distanceFromSpeaker = Math.sqrt(Math.pow(distanceFromAprilTag, 2) + Math.pow(2.340102, distanceFromAprilTag));

        double distance_min = 0;
        double distance_max = 0;

        double out_min = 0;
        double out_max = 0;

        return (distanceFromSpeaker - distance_min) * (out_max - out_min) / (distance_max - distance_min) + out_min;
    }
}
