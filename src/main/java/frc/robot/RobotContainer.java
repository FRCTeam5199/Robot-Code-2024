// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autos;
import frc.robot.commands.AprilTag.PoseEstimation;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.commands.AprilTag.DriveToAMP;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.UserInterface;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ClimberSubsystem climberSubsystem = new ClimberSubsystem();

   Autos auton;

   private double MaxSpeed = 6; // 6 meters per second desired top speed
   private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
 
   /* Setting up bindings for necessary control of the swerve drive platform */
   private final CommandXboxController commandXboxController = new CommandXboxController(MainConstants.OperatorConstants.CONTROLLER_PORT); // My joystick
   private final SwerveDrive drivetrain = TunerConstants.DriveTrain; // My drivetrain
   private final AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
 
   private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
       .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
       .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                // driving in open loop
   private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
   private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
   private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    ConditionalCommand climbCommandGroup = 
      new ConditionalCommand(
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> intakeSubsystem.setIntakeAngle(0))
            // new WaitCommand(0.8),
            // new InstantCommand(() -> arm.extend()),
            // new InstantCommand(() -> elevator.high())
          ),
          new SequentialCommandGroup(
              // new InstantCommand(() -> wrist.moveRight()),
              // new WaitCommand(0.5),
              // new InstantCommand(() -> wrist.stopRotation())
          )
        ),
        new ParallelCommandGroup(
          new InstantCommand(() -> climberSubsystem.storeClimb())
        ),
      climberSubsystem::isClimbing);
      
    // Drive
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    // reset the field-centric heading by pressing start button/hamburger menu button
    commandXboxController.start().onTrue(drivetrain.runOnce(()->drivetrain.seedFieldRelative()));

    commandXboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    commandXboxController.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-commandXboxController.getLeftY(), -commandXboxController.getLeftX()))));



      aprilTagSubsystem.setDefaultCommand(aprilTagSubsystem.updateRobotPose());
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // Intake
    commandXboxController.leftTrigger().onTrue(intakeSubsystem.setIntakeSpeed(1))/*.onFalse(intakeSubsystem.setIntakeSpeed(0))*/;

    // Climber
    commandXboxController.rightBumper().onTrue(climbCommandGroup);
  }

  public RobotContainer() {
      auton = new Autos(drivetrain);
      configureBindings();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auton.twoPieceExtendedRed();
  }
}
  