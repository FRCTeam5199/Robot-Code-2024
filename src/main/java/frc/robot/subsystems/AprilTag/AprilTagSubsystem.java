// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AprilTag;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class AprilTagSubsystem extends SubsystemBase {

    public MainConstants Constants;

    public static PhotonCamera frontCamera;
    public static PhotonCamera leftCamera;
    public static PhotonCamera rigthCamera;
    public static PhotonCamera backCamera;


    // public static PhotonCamera[] cameraDirections = {front, left, rigth, back};

  public AprilTagSubsystem(MainConstants Constants) {
    this.Constants = Constants;
    frontCamera = new PhotonCamera(Constants.cameraNames[0]);
    leftCamera = new PhotonCamera(Constants.cameraNames[1]);    
    rigthCamera = new PhotonCamera(Constants.cameraNames[2]);
    backCamera = new PhotonCamera(Constants.cameraNames[3]);

    // for(int i = 0; i <= Constants.cameraNames.length; i++){
    //   cameraDirections[i] = new PhotonCamera(Constants.cameraNames[i]);
    // }
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
    // This method will be called once per scheduler run
  }
  public void init(){

  }

  //calculate angle of shooter for speaker on both alliances
  public double speakers(){
    double angle = 0;
        if (getAllianceColor().equals("Red")){
          double b = 4.0; 
          double c = Constants.TARGET_HEIGHT_METERS;
        
        // Calculate the angle A using arccosine (inverse cosine)
        angle = Math.toDegrees(Math.acos(b / c));
        
        // Displaying the calculated angle A
        }
        else if (getAllianceColor().equals("Blue")){

        }
        return angle;
      }
  }
