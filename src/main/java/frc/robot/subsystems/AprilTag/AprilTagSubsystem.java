// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AprilTag;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AbstractConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;


public class AprilTagSubsystem extends SubsystemBase {

    public AbstractConstants Constants;

    public static PhotonCamera frontCamera;
    public static PhotonCamera leftCamera;
    public static PhotonCamera rigthCamera;
    public static PhotonCamera backCamera;


    // public static PhotonCamera[] cameraDirections = {front, left, rigth, back};

  public AprilTagSubsystem(AbstractConstants Constants) {
    this.Constants = Constants;
    frontCamera = new PhotonCamera(Constants.cameraNames[0]);
    leftCamera = new PhotonCamera(Constants.cameraNames[1]);    
    rigthCamera = new PhotonCamera(Constants.cameraNames[2]);
    backCamera = new PhotonCamera(Constants.cameraNames[3]);

    // for(int i = 0; i <= Constants.cameraNames.length; i++){
    //   cameraDirections[i] = new PhotonCamera(Constants.cameraNames[i]);
    // }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void init(){

  }
  public void getEstimatedFront(){
          
      }
  }
