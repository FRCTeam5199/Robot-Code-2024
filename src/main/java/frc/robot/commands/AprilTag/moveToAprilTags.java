// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTag;


import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTag.AprilTagSubsystem;
import frc.robot.subsystems.drivetrain.swerveDrive.SwerveDrive;

import frc.robot.constants.AbstractConstants;
import frc.robot.constants.MainConstants;

public class moveToAprilTags extends Command {
  /** Creates a new moveToAprilTags. */
  public double ANGULAR_P;
  public double ANGULAR_D;
  public double rotationSpeed;
  public PIDController turnController;
  public double P_GAIN;
  public double D_GAIN;
  PIDController controller;

  public double forwardSpeed;


  public SwerveDrive swerveDrive;
  public static AprilTagSubsystem cameras;
  public static AbstractConstants constants;

  public moveToAprilTags(SwerveDrive SwerveDrive, AprilTagSubsystem aprilTagSubsystem, AbstractConstants constants) {
    this.swerveDrive = swerveDrive;
    this.cameras = aprilTagSubsystem;
    this.constants = constants;

    addRequirements(SwerveDrive, aprilTagSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }


  @Override
  public void initialize() {
  P_GAIN = 0.1;
  D_GAIN = 0;
  controller = new PIDController(P_GAIN, 0, D_GAIN);

  ANGULAR_P = 0.1;
  ANGULAR_D = 0.0;
  turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = cameras.frontCamera.getLatestResult(); 

    if (result.hasTargets()) {
        rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        double range = PhotonUtils.calculateDistanceToTargetMeters(
                                constants.FRONT_CAMERA_HEIGHT_METERS,
                                result.getBestTarget().getBestCameraToTarget().getZ(),
                                constants.FRONT_CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                forwardSpeed = -controller.calculate(range, constants.GOAL_RANGE_METERS);


        forwardSpeed = -controller.calculate(range, constants.TARGET_HEIGHT_METERS);

    } else {
        rotationSpeed = 0;
        forwardSpeed = 0;
    }
    
    // swerveDrive.setDefaultCommand(aprilTagDrive);
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
