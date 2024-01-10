// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class AprilTagSubsystem extends SubsystemBase {

    public MainConstants Constants = new MainConstants();


    public static PhotonCamera frontCamera;
    public static PhotonCamera leftCamera;
    public static PhotonCamera rigthCamera;
    public static PhotonCamera backCamera;
   // 0 Front, 1 Back, 2 Left, 3 Rigggggggggggggggght
    public PhotonCamera[] allCameras = {frontCamera, backCamera, leftCamera, backCamera};
    public PhotonTrackedTarget[] bestTargetFromCameras;

    // public static PhotonCamera[] cameraDirections = {front, left, rigth, back};

  public AprilTagSubsystem() {
    for(int i = 0; i <= Constants.cameraNames.length; i++){
      allCameras[i] = new PhotonCamera(Constants.cameraNames[i]);
    }
    
    PhotonCamera allCameras[] = {frontCamera, backCamera, rigthCamera, leftCamera};
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

  public void init(){
    AprilTagFieldLayout AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

  @Override
  public void periodic() {
    bestTargetFromCameras[0] = frontCamera.getLatestResult().getBestTarget();
    bestTargetFromCameras[1] = backCamera.getLatestResult().getBestTarget();
    bestTargetFromCameras[2] = leftCamera.getLatestResult().getBestTarget();
    bestTargetFromCameras[3] = rigthCamera.getLatestResult().getBestTarget();

    

    // This method will be called once per scheduler run
  }
  public void init(){
    
    AprilTagFieldLayout AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  }

  public boolean checkCameraForSpecificID(int ID){
        int desiredTagID = ID;
        for (PhotonTrackedTarget tag : getAllTargets()) {
            if (tag.getFiducialId() == desiredTagID) {
                return true;
            }
        }
        return false;
  }
  public PhotonTrackedTarget getOneTag(int ID){
      int desiredTagID = ID;
      if(checkCameraForSpecificID(ID)){
        for (PhotonTrackedTarget tag : getAllTargets()) {
            if (tag.getFiducialId() == desiredTagID) {
                return tag;
            }
        }
      }
        return null;
  }

  public Command printAngle(){
    return run(()-> System.out.println(speakersAligning()));
    //balls, big ones
  }
  
  //calculate angle of shooter for speaker on both alliances
  public double speakers(){
    double angle = 0;
    double distance = 4;
        if (getAllianceColor().equals("Red")){
          if(checkCameraForSpecificID(4)){
           distance = 4 ; 
          }
          double c = Constants.TARGET_HEIGHT_METERS;
        
        // Calculate the angle A using arccosine (inverse cosine)
        angle = Math.toDegrees(Math.acos(distance / c));
        
        // Displaying the calculated angle A
        }
        else if (getAllianceColor().equals("Blue")){

        }
        angleForShooter = Math.toDegrees(Math.tan(speakerHeight/distanceFromRobot));
        System.out.println("//////////////////////" + angleForShooter);
        return angleForShooter;
      }  
}
