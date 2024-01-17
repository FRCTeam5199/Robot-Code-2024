
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = limelight.getEntry("tx");
    NetworkTableEntry ty = limelight.getEntry("ty");
    NetworkTableEntry tz = limelight.getEntry("tz");
    double x = tx.getDouble(0);
    double y = ty.getDouble(0);
    double z = tz.getDouble(0);

    SwerveDrive drive = TunerConstants.DriveTrain;


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

    public PhotonPoseEstimator ambiguityCheck(PhotonCamera back) {
        double[] ambiguity = new double[4];
        PhotonCamera[] cameras = new PhotonCamera[]{back};
        ambiguity[0] = back.getLatestResult().getMultiTagResult().estimatedPose.ambiguity;
        //ambiguity[1] = left.getLatestResult().getMultiTagResult().estimatedPose.ambiguity;
        //ambiguity[2] = right.getLatestResult().getMultiTagResult().estimatedPose.ambiguity;
        //ambiguity[3] = back.getLatestResult().getMultiTagResult().estimatedPose.ambiguity;

        OptionalDouble lowestambiguity = Arrays.stream(ambiguity).sorted().findFirst();
        if (lowestambiguity.isPresent()) {
          //  for (int i = 4; i <= ambiguity.length; i++) {
                if (cameras[3].getLatestResult().getMultiTagResult().estimatedPose.isPresent && cameras[3].getLatestResult().getMultiTagResult().estimatedPose.ambiguity == ambiguity[0]) {
                    return new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras[0], Constants.cameraPositions[3]);
                }

        }
        if (back.getLatestResult().getMultiTagResult().estimatedPose.isPresent) {
            System.out.println(back.getLatestResult().getMultiTagResult().estimatedPose.ambiguity);
            return new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Back"), Constants.cameraPositions[0]);
        }
        System.out.println("1 Tag");
        return new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Back"), Constants.cameraPositions[0]);
    }

    public Command updatePose() {
        poseEstimator = ambiguityCheck(allCameras[3]);

        Optional<EstimatedRobotPose> bool = poseEstimator.update();
        if (bool.isPresent()) {
            if(bool.get().estimatedPose.toPose2d() != testthing()) {
                return runOnce(() -> System.out.println(bool.get().estimatedPose.getTranslation()));
            }
        }else {
            return runOnce(() -> System.out.println("nothing present"));
        }
        return null;
    }

    public Pose2d testthing() {
        poseEstimator = ambiguityCheck(allCameras[3]);

        Optional<EstimatedRobotPose> bool = poseEstimator.update();
        return bool.map(estimatedRobotPose -> estimatedRobotPose.estimatedPose.toPose2d()).orElse(null);
    }



    // This method will be called once per scheduler run
    public Command updateRobotPose(){
        poseEstimator = ambiguityCheck(allCameras[3]);
        Optional<EstimatedRobotPose> bool = poseEstimator.update();
        return bool.map(estimatedRobotPose -> runOnce(() -> drive.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), Timer.getFPGATimestamp()))).orElse(runOnce(()->System.out.println("no visible tag")));
    }
}
