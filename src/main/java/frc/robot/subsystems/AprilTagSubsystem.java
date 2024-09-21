// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utility.CommandXboxController;

public class AprilTagSubsystem extends SubsystemBase {
    private final SwerveDrive drivetrain = TunerConstants.DriveTrain;
    public AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public SwerveRequest.FieldCentric driveHeading = new SwerveRequest.FieldCentric();
    public PIDController aim = new PIDController(.07, 0.00, .0);
    private PhotonCamera frontCamera;
    private PhotonCamera backCamera;
    private PhotonPoseEstimator frontPhotonEstimator;
    private PhotonPoseEstimator backPhotonEstimator;
    private double lastEstTimestamp = 0;
    private PhotonPipelineResult lastResult;
    private CommandXboxController mainCommandXboxController = new CommandXboxController(MainConstants.CONTROLLER_PORT);

    public AprilTagSubsystem() {
        frontCamera = new PhotonCamera("Front");
        backCamera = new PhotonCamera("Back");

        frontPhotonEstimator =
                new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera, MainConstants.cameraPositions[0]);
        frontPhotonEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        backPhotonEstimator =
                new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera, MainConstants.cameraPositions[3]);
        backPhotonEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
    }

    public double getDistanceFromRedSpeaker() {
        return Math.sqrt(Math.pow((16.579342 - drivetrain.getPose().getX()), 2)
                + Math.pow((5.547867999 - drivetrain.getPose().getY()), 2));
    }

    public void getLatestResultFront() {
        lastResult = frontCamera.getLatestResult();
    }

    public void getLatestResultBack() {
        lastResult = backCamera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFront() {
        frontPhotonEstimator.setReferencePose(drivetrain.getPose());

        getLatestResultFront();


        // filtering stages
        // Ensure the result is
        if (lastResult.getTimestampSeconds() <= lastEstTimestamp) {
            return Optional.empty();
        } else if (lastResult.getTargets().size() < 2) {
            return Optional.empty();
        } else {
            lastEstTimestamp = lastResult.getTimestampSeconds();
            return frontPhotonEstimator.update(lastResult);
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseBack() {
        backPhotonEstimator.setReferencePose(drivetrain.getPose());

        getLatestResultBack();


        // filtering stages
        // Ensure the result is
        if (lastResult.getTimestampSeconds() <= lastEstTimestamp) {
            return Optional.empty();
        } else if (lastResult.getTargets().size() < 2) {
            return Optional.empty();
        } else {
            lastEstTimestamp = lastResult.getTimestampSeconds();
            return backPhotonEstimator.update(lastResult);
        }
    }

    public double getTimestamp() {
        return lastResult.getTimestampSeconds();
    }

    public Command speakerAlignmentRed() {
        return drivetrain.applyRequest(() -> driveHeading.withVelocityX(-mainCommandXboxController.getLeftY())
                .withVelocityY(-mainCommandXboxController.getLeftX()).withRotationalRate(
                        aim.calculate(drivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(-3)).getDegrees(),
                                Units.radiansToDegrees(
                                        Math.atan((5.59 - drivetrain.getPose().getY()) / (16.58 - drivetrain.getPose().getX()))))));
    }


    public Command speakerAlignmentBlue() {
//        target = new TrapezoidProfile.State(Math.atan((5.548 - drivetrain.getPose().getY()) / (-.0381 - drivetrain.getPose().getX())), 0);
//        betterAim = new TrapezoidProfile(aimConstraints);
//        var goToTarget = betterAim.calculate(1, target, new TrapezoidProfile.State(drivetrain.getPose().getRotation().getRadians(), 1));
//
//        return drivetrain.applyRequest(() -> driveHeading.withVelocityX(-mainCommandXboxController.getLeftY()).withVelocityY(-mainCommandXboxController.getLeftX()).withRotationalRate(aim.calculate(drivetrain.getPose().getRotation().getRadians(), goToTarget.position)));

        return drivetrain.applyRequest(() -> driveHeading.withVelocityX(-mainCommandXboxController.getLeftY()).withVelocityY(-mainCommandXboxController.getLeftX()).withRotationalRate(aim.calculate((drivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees()), Units.radiansToDegrees(Math.atan((5.48 - drivetrain.getPose().getY()) / (-0.0381 - drivetrain.getPose().getX()))))));
    }

    public Rotation2d autonAimRed() {
        Pose2d stagePoseRed = new Pose2d(16.579342, 5.547867999, new Rotation2d(180));
        Pose2d stagePoseBlue = new Pose2d(-0.038099999999999995, 5.547867999, new Rotation2d(0));
        return new Rotation2d(Math.atan((5.548 - drivetrain.getPose().getY()) / (16.58 - drivetrain.getPose().getX())));
    }

    public Rotation2d autonAimBlue() {
        return new Rotation2d(Math.atan((5.548 - drivetrain.getPose().getY()) / (-0.0381 - drivetrain.getPose().getX()))).plus(Rotation2d.fromDegrees(180));
    }
}
