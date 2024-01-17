
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
    PhotonPoseEstimator poseEstimator;
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


    public PhotonPoseEstimator ambiguityCheck(PhotonCamera four) {
        double[] result = new double[4];
        MultiTargetPNPResult[] pnpResult = new MultiTargetPNPResult[4];
        //result[0] = one.getLatestResult().getMultiTagResult().estimatedPose.ambiguity;
        //result[1] = two.getLatestResult().getMultiTagResult().estimatedPose.ambiguity;
        //result[2] = three.getLatestResult().getMultiTagResult().estimatedPose.ambiguity;
        result[3] = four.getLatestResult().getMultiTagResult().estimatedPose.ambiguity;

        OptionalDouble lowestambiguity = Arrays.stream(result).sorted().findFirst();
      /*
      if (lowestambiguity.isPresent()) {
          if (one.getLatestResult().getMultiTagResult().estimatedPose.ambiguity == lowestambiguity.getAsDouble()) {
              return new PhotonPoseEstimator(fieldLayout, poseStrategy, one, Constants.cameraPositions[0]);
          } else {
              if (two.getLatestResult().getMultiTagResult().estimatedPose.ambiguity == lowestambiguity.getAsDouble()) {
                  return new PhotonPoseEstimator(fieldLayout, poseStrategy, two, Constants.cameraPositions[1]);
              } else {
                  if (three.getLatestResult().getMultiTagResult().estimatedPose.ambiguity == lowestambiguity.getAsDouble()) {
                      return new PhotonPoseEstimator(fieldLayout, poseStrategy, three, Constants.cameraPositions[2]);
                  } else {
                      if (four.getLatestResult().getMultiTagResult().estimatedPose.ambiguity == lowestambiguity.getAsDouble()) {
                          return new PhotonPoseEstimator(fieldLayout, poseStrategy, four, Constants.cameraPositions[3]);
                      }
                  }
              }
          }
      }

       */
        if (four.getLatestResult().getMultiTagResult().estimatedPose.isPresent) {
            System.out.println(four.getLatestResult().getMultiTagResult().estimatedPose.ambiguity);
            return new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR , new PhotonCamera("Back"), Constants.cameraPositions[0]);
        }else {
            System.out.println("1 Tag");
            return new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Back"), Constants.cameraPositions[0]);
        }
    }

    public Command updatePose() {
        poseEstimator = ambiguityCheck(allCameras[3]);
        Optional<EstimatedRobotPose> bool = poseEstimator.update();
        if(bool.isPresent()){
            return runOnce(()->System.out.println(bool.get().estimatedPose.getTranslation()));
        }
        return runOnce(()->System.out.println("asfhorjogi"));
    }

    public Optional<EstimatedRobotPose> getVisionPose(){
        poseEstimator = ambiguityCheck(allCameras[3]);
        System.out.println("Using Vision");
        return poseEstimator.update();
    }


}
