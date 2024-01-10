// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTag;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.constants.MainConstants;
import frc.robot.constants.MainConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PoseEstimation extends Command {
  /** Creates a new PoseEstimation. */
  public MainConstants Constants = new MainConstants();

  public AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();

  public SwerveDrive drivetrain;

  public AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  private static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

    
  private static final Transform3d FRONT_CAMERA_TO_CENTER = new Transform3d(
        new Translation3d(Units.inchesToMeters(5.800), Units.inchesToMeters(-8.517), Units.inchesToMeters(43.3)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(35.895 / 2)));

  private static final Transform3d BACK_CAMERA_TO_CENTER = new Transform3d(
      new Translation3d(Units.inchesToMeters(5.760), Units.inchesToMeters(-12.707), Units.inchesToMeters(43.3)),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-35.895 / 2)));

  private static final Transform3d LEFT_CAMERA_TO_CENTER = new Transform3d(
      new Translation3d(Units.inchesToMeters(5.800), Units.inchesToMeters(-8.517), Units.inchesToMeters(43.3)),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(35.895 / 2)));

  private static final Transform3d RIGHT_CAMERA_TO_CENTER = new Transform3d(
      new Translation3d(Units.inchesToMeters(5.760), Units.inchesToMeters(-12.707), Units.inchesToMeters(43.3)),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-35.895 / 2)));

  private final SwerveDrivePoseEstimator poseEstimator;


  public OriginPosition originPosition = AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;

  public final PhotonPoseEstimator photonPoseEstimatorFront;
  public final PhotonPoseEstimator photonPoseEstimatorBack;
  // public final PhotonPoseEstimator photonPoseEstimatorLeft;
  // public final PhotonPoseEstimator photonPoseEstimatorRight;

  public PoseEstimation(SwerveDrive drivetrain) {
    this.drivetrain = drivetrain;

    poseEstimator = new SwerveDrivePoseEstimator(Constants.SWERVE_KINEMATICS ,drivetrain.getPose().getRotation(), drivetrain.getModulePositions() , new Pose2d(0, 0,drivetrain.getPose().getRotation()), STATE_STDS , VISION_STDS);

  photonPoseEstimatorFront = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagSubsystem.frontCamera , FRONT_CAMERA_TO_CENTER);
  photonPoseEstimatorBack = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagSubsystem.backCamera, BACK_CAMERA_TO_CENTER);
  // photonPoseEstimatorLeft = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,aprilTagSubsystem.leftCamera, LEFT_CAMERA_TO_CENTER);
  // photonPoseEstimatorRight = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,aprilTagSubsystem.rigthCamera, RIGHT_CAMERA_TO_CENTER);

  photonPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  photonPoseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // Use addRequirements() here to declare subsystem dependencies.
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Optional<EstimatedRobotPose> optionalEstimatedPoseFront = photonPoseEstimatorFront.update();
  if (optionalEstimatedPoseFront.isPresent()) {
    final EstimatedRobotPose estimatedPose = optionalEstimatedPoseFront.get();          
    poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    
  }

  final Optional<EstimatedRobotPose> optionalEstimatedPoseBack = photonPoseEstimatorBack.update();
  if (optionalEstimatedPoseBack.isPresent()) {
    final EstimatedRobotPose estimatedPose = optionalEstimatedPoseBack.get();          
    poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
  }
  poseEstimator.getEstimatedPosition().getTranslation();

  // final Optional<EstimatedRobotPose> optionalEstimatedPoseLeft = photonPoseEstimatorLeft.update();
  // if (optionalEstimatedPoseLeft.isPresent()) {
  //   final EstimatedRobotPose estimatedPose = optionalEstimatedPoseLeft.get();          
  //   poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
  // }
  // poseEstimator.getEstimatedPosition().getTranslation();
  // }
  // final Optional<EstimatedRobotPose> optionalEstimatedPoseRight = photonPoseEstimatorRight.update();
  // if (optionalEstimatedPoseRight.isPresent()) {
  //   final EstimatedRobotPose estimatedPose = optionalEstimatedPoseRight.get();          
  //   poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
  // }
  // poseEstimator.getEstimatedPosition().getTranslation();
  // }
  }
  
  public SwerveDrivePoseEstimator getPoseEstimator(){
    return poseEstimator;
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
