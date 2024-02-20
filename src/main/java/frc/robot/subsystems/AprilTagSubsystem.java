
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
import java.util.Objects;
import java.util.Optional;

import frc.robot.utility.CommandXboxController;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class AprilTagSubsystem implements Subsystem {

    public MainConstants Constants = new MainConstants();
    EstimatedRobotPose[] robotPose = new EstimatedRobotPose[4];
    SwerveDrive drive = TunerConstants.DriveTrain;
    private final CommandXboxController mainCommandXboxController = new CommandXboxController(MainConstants.OperatorConstants.MAIN_CONTROLLER_PORT);


    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = limelight.getEntry("tx");
    NetworkTableEntry ty = limelight.getEntry("ty");
    NetworkTableEntry tz = limelight.getEntry("tz");
    double x = tx.getDouble(0);
    double y = ty.getDouble(0);
    double z = tz.getDouble(0);

    AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    PhotonPoseEstimator.PoseStrategy poseStrategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    PhotonPoseEstimator poseEstimatorFront = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Front"), Constants.cameraPositions[0]);

    // PhotonPoseEstimator multiPoseEstimatorRight = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Right"), Constants.cameraPositions[1]);
    // PhotonPoseEstimator singlePoseEstimatorRight = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Right"), Constants.cameraPositions[1]);

    // PhotonPoseEstimator multiPoseEstimatorLeft = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Left"), Constants.cameraPositions[3]);
    // PhotonPoseEstimator singlePoseEstimatorLeft = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Left"), Constants.cameraPositions[3]);

    // PhotonPoseEstimator multiPoseEstimatorBack = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Back"), Constants.cameraPositions[3]);
    // PhotonPoseEstimator singlePoseEstimatorBack = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Back"), Constants.cameraPositions[3]);

    // PhotonPoseEstimator multiPoseEstimatorShooter = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Shooter"), Constants.cameraPositions[4]);
    // PhotonPoseEstimator singlePoseEstimatorShooter = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, new PhotonCamera("Shooter"), Constants.cameraPositions[4]);

    public static PhotonCamera frontCamera;
    // public static PhotonCamera leftCamera;
    // public static PhotonCamera rightCamera;
    // public static PhotonCamera backCamera;
    // public static PhotonCamera shooter;

    // 0 Front, 1 Back, 2 Left, 3 Rigggggggggggggggght
    public PhotonCamera[] allCameras = {frontCamera};
    public PhotonTrackedTarget[] bestTargetFromCameras;
    public MultiTargetPNPResult[] multiTargetPNPResults;
    PIDController aimControl;

    // public static PhotonCamera[] cameraDirections = {front, left, rigth, back};

    public AprilTagSubsystem() {
        allCameras[0] = new PhotonCamera("Front");
        poseEstimatorFront.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        // allCameras[3] = new PhotonCamera("Back");
        // allCameras[4] = new PhotonCamera("Shooter");
        aimControl = new PIDController(1, .1, 0);
    }

    /**
     * get alliance
     * 
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
     * estimated position from front
     */

    public Optional<EstimatedRobotPose> getVisionPoseFront() {
        poseEstimatorFront.setReferencePose(drive.getPose());

        var result = allCameras[0].getLatestResult();

        if(result.hasTargets()){
          return poseEstimatorFront.update();
         }else{
            // System.out.println("O Tags on Front");
            return Optional.empty();
         }
    }

    /**
     * estimated pose right
     * @return the estimated vision pose from the right camera. If there is no position to give it returns an empty value
     */
    public Optional<EstimatedRobotPose> getVisionPoseRight() {
        // var result = allCameras[1].getLatestResult();
        // if(result.getMultiTagResult().estimatedPose.isPresent && result.getMultiTagResult().estimatedPose.ambiguity < .2){
        //     return multiPoseEstimatorRight.update();
        // }else if(result.hasTargets()){
        //     return singlePoseEstimatorRight.update();

        // }else{
            // System.out.println("O Tags On Right");
            return Optional.empty();
        // }
    }

    /**
     * estimated pose left
     */
    public Optional<EstimatedRobotPose> getVisionPoseLeft() {
        // var result = allCameras[2].getLatestResult();
        // if(result.getMultiTagResult().estimatedPose.isPresent && result.getMultiTagResult().estimatedPose.ambiguity < .2){
            // return multiPoseEstimatorLeft.update();
        // }else if(result.hasTargets()){
            // return singlePoseEstimatorLeft.update();

        // }else{
            // System.out.println("O Tags On Left");
            return Optional.empty();
        // }
    }

    /**
     * estimated pose back
     */
    public Optional<EstimatedRobotPose> getVisionPoseBack() {
        // var result = allCameras[3].getLatestResult();
        // if(result.getMultiTagResult().estimatedPose.isPresent && result.getMultiTagResult().estimatedPose.ambiguity < .2){
        //     // System.out.println("2 Tags Back");
        //     return multiPoseEstimatorBack.update();
        // }else if(result.hasTargets()){
        //     // System.out.println("1 Tag Back");
        //     return singlePoseEstimatorBack.update();
        // }else{
            // System.out.println("O Tags Back");
            return Optional.empty();
        // }
    }

    /**
     * shooter gamera pose estimation
     */
    public Optional<Pose3d> getShooterVision(){
    //    PhotonPipelineResult result = allCameras[4].getLatestResult();
    //    if(singlePoseEstimatorShooter.update().isPresent())
    //        return Optional.ofNullable(singlePoseEstimatorShooter.update().get().estimatedPose);
    //     else{
            return Optional.empty();
    //     }

    }

    /**
     * Sets the robots heading to align with the goal based on the position of the bot on the field.
     *
     *
     * @param x pass the speed in the x direction
     * @param y pass the speed in the y direction
     *
     */
    public Command speakerAlignment(double x, double y){
        PIDController aim = new PIDController(.1, 0, 0);
        SwerveRequest.FieldCentric driveHeading = new SwerveRequest.FieldCentric();
        Pose2d stagePoseRed = new Pose2d(16.579342, 5.547867999, new Rotation2d(180));
        Pose2d stagePoseBlue = new Pose2d(-0.038099999999999995, 5.547867999, new Rotation2d(0));
        Translation2d target = stagePoseRed.getTranslation().minus(drive.getPose().getTranslation());
        double targetHeading = Units.radiansToDegrees(Math.atan((5.54787 - drive.getPose().getY())/(16.58 - drive.getPose().getX())));
        return drive.applyRequest(()-> driveHeading.withVelocityX(-mainCommandXboxController.getLeftY()).withVelocityY(-mainCommandXboxController.getLeftX()).withRotationalRate(aim.calculate(drive.getPose().getRotation().getDegrees(), Units.radiansToDegrees(Math.atan((5.54 - drive.getPose().getY())/(16.58 - drive.getPose().getX()))))));

    }

    public double targetHeading(){
        Pose2d stagePoseRed = new Pose2d(16.579342, 5.547867999, new Rotation2d(180));
        Pose2d stagePoseBlue = new Pose2d(-0.038099999999999995, 5.547867999, new Rotation2d(0));

        double distanceX = stagePoseRed.getX() - drive.getPose().getX();
        double distanceY = stagePoseRed.getY() - drive.getPose().getY();
        return Units.radiansToDegrees(Math.atan(distanceY/distanceX));

    }


    // public void shooterAlign(double speedX, double speedY){
    //     PhotonCamera targetCam = null;
    //     if(Objects.equals(getAllianceColor(), "Red")){
            //for(int i = 0; i <= allCameras.length; i++){
                //if(allCameras[i].getLatestResult().getMultiTagResult().fiducialIDsUsed.contains(3) || allCameras[i].getLatestResult().getMultiTagResult().fiducialIDsUsed.contains(4) || allCameras[i].getLatestResult().getBestTarget().getFiducialId() == 3 || allCameras[i].getLatestResult().getBestTarget().getFiducialId() == 4){
                  //  targetCam = allCameras[i];
                  //  break;
                //}
                //if(i == 4){
                //    i = -1;
              //  }
            //}
            // targetCam = allCameras[3];
    //         while(Objects.equals(targetCam.getName(), "Right") || Objects.equals(targetCam.getName(), "Left")) {
    //             drive.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 1)));
    //             if(allCameras[0].getLatestResult().getBestTarget().getFiducialId() == 3 || allCameras[0].getLatestResult().getBestTarget().getFiducialId() == 4){
    //                 targetCam = allCameras[0];
    //                 break;
    //             }
    //             if(allCameras[3].getLatestResult().getBestTarget().getFiducialId() == 3 || allCameras[3].getLatestResult().getBestTarget().getFiducialId() == 4){
    //                 targetCam = allCameras[3];
    //                 break;
    //             }
    //         }
    //         if (targetCam.getLatestResult().hasTargets() && targetCam.getLatestResult().getBestTarget().getFiducialId() == 3) {
    //             drive.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(speedX, speedY, aimControl.calculate(targetCam.getLatestResult().getBestTarget().getYaw(), 10) * .05)));
    //         }
    //         if (targetCam.getLatestResult().hasTargets() && targetCam.getLatestResult().getBestTarget().getFiducialId() == 4) {
    //             drive.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(speedX, speedY, aimControl.calculate(targetCam.getLatestResult().getBestTarget().getYaw(), 0) * .05)));
    //         }
    //     }
    //     if(getAllianceColor() == "Blue"){
    //         for(int i = 0; i <= allCameras.length; i++){
    //             if(allCameras[i].getLatestResult().getMultiTagResult().fiducialIDsUsed.contains(7) || allCameras[i].getLatestResult().getMultiTagResult().fiducialIDsUsed.contains(8) || allCameras[i].getLatestResult().getBestTarget().getFiducialId() == 7 || allCameras[i].getLatestResult().getBestTarget().getFiducialId() == 8){
    //                 targetCam = allCameras[i];
    //                 break;
    //             }
    //             if(i == 4){
    //                 i = -1;
    //             }
    //         }
    //         while(Objects.equals(targetCam.getName(), "Right") || Objects.equals(targetCam.getName(), "Left")) {
    //             drive.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 1)));
    //             if(allCameras[0].getLatestResult().getBestTarget().getFiducialId() == 7 || allCameras[0].getLatestResult().getBestTarget().getFiducialId() == 8){
    //                 targetCam = allCameras[0];
    //                 break;
    //             }
    //             if(allCameras[3].getLatestResult().getBestTarget().getFiducialId() == 7 || allCameras[3].getLatestResult().getBestTarget().getFiducialId() == 8){
    //                 targetCam = allCameras[3];
    //                 break;
    //             }
    //         }
    //         if(targetCam.getLatestResult().getBestTarget().getFiducialId() == 7){
    //             drive.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(0, 0, aimControl.calculate(targetCam.getLatestResult().getBestTarget().getYaw(), 10)* .06)));
    //         }
    //         if(targetCam.getLatestResult().getBestTarget().getFiducialId() == 8){
    //             drive.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(0, 0, aimControl.calculate(targetCam.getLatestResult().getBestTarget().getYaw(), 0)* .06)));
    //         }
    //     }


    // }
    // public Command alignSpeaker(){
    //     return run(()-> speakersAligning());
    // }

    // public double speakersAligning(){
    // double angleForShooter = 0;
    // double speakerHeight = MainConstants.SPEAKER_Z - MainConstants.ARM_PIVOT_Z;
    // // slightly in front of april tag so it doesnt aim out of field
    // double distanceFromRobot = 0;
    //     if (getAllianceColor().equals("Blue")){
    //     //   distanceFromRobot = poseEstimation.getPoseEstimator().getEstimatedPosition().getTranslation().getDistance(new Translation2d(2, 218.42));
    //         distanceFromRobot = drive.getPose().getTranslation().getDistance(new Translation2d(2, 218.42));
    //     }
    //     else if (getAllianceColor().equals("Red")){
    //     // distanceFromRobot = drive.getPose().getTranslation().getDistance(new Translation2d(16.579342, 5.5478679999999999));
    //     distanceFromRobot = new Translation2d(14.700 - 1.1049, 5.54).getDistance(new Translation2d(16.579342, 5.54));
    //     }

    //     distanceFromRobot += MainConstants.ARM_PIVOT_X_OFFSET;
    //     angleForShooter = Math.toDegrees(Math.atan(speakerHeight/distanceFromRobot));
    //     System.out.println("angle:   " + angleForShooter);

//         return angleForShooter;
//       }


}
