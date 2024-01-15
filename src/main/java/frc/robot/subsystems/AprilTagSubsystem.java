
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.MainConstants;

import java.util.Optional;

import frc.robot.subsystems.drivetrain.SwerveDrive;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class AprilTagSubsystem implements Subsystem {

    public MainConstants Constants = new MainConstants();
    PhotonPoseEstimator[] poseEstimator;
    EstimatedRobotPose[] robotPose = new EstimatedRobotPose[4];
    SwerveDrive drive;

    AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    PhotonPoseEstimator.PoseStrategy poseStrategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
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
    for(int i = 0; i <= Constants.cameraNames.length; i++){
      allCameras[i] = new PhotonCamera(Constants.cameraNames[i]);
      poseEstimator[i] = new PhotonPoseEstimator(fieldLayout, poseStrategy, allCameras[i], Constants.cameraPositions[i]);

    }

  }

  public String getAllianceColor(){
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



  @Override
  public void periodic() {

      for(int i = 0; i <= 3; i++){
          poseEstimator[i].setReferencePose(drive.getPose());
          if(allCameras[i].getLatestResult().getMultiTagResult().estimatedPose.isPresent) {
              multiTargetPNPResults[i] = allCameras[i].getLatestResult().getMultiTagResult();
              if(poseEstimator[i].update().isPresent()){
                  robotPose[i] = new EstimatedRobotPose(getVisionPose()[i], Timer.getFPGATimestamp(), allCameras[i].getLatestResult().getTargets(), PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
              }
          }
      }
    bestTargetFromCameras[0] = frontCamera.getLatestResult().getBestTarget();
    bestTargetFromCameras[1] = backCamera.getLatestResult().getBestTarget();
    bestTargetFromCameras[2] = leftCamera.getLatestResult().getBestTarget();
    bestTargetFromCameras[3] = rightCamera.getLatestResult().getBestTarget();



    double[] poseAmbiguity = new double[]{0, 0, 0, 0};

    for(int i = 0; i <=3; i++){
        poseAmbiguity[i] = bestTargetFromCameras[i].getPoseAmbiguity();
    }

    for(int i = 0; i <= 3; i++){
        if(poseAmbiguity[i] > .2){
            poseAmbiguity[i] = 0;
        }
    }


    

    // This method will be called once per scheduler run
  }

    public Pose3d[] getVisionPose(){
      Pose3d[] posees = new Pose3d[4];
      for(int i = 0; i <=3; i++) {
          posees[i] = poseEstimator[i].update().get().estimatedPose;
      }
      return posees;
    }

    public Pose2d robotPose(){
      return robotPose[0].estimatedPose.toPose2d();
    }

}
