// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

        public final static ArmSubsystem arm = new ArmSubsystem();
        public final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
        public final static IntakeSubsystem intake = new IntakeSubsystem();
        private final XboxController driveXboxController = new XboxController(0);
        private final double MaxSpeed = 6; // 6 meters per second desired top speed
        private final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
        /* Setting up bindings for necessary control of the swerve drive platform */
        private final CommandXboxController commandXboxController = new CommandXboxController(
                        MainConstants.OperatorConstants.CONTROLLER_PORT); // My joystick
        private final SwerveDrive drivetrain = TunerConstants.DriveTrain; // My drivetrain
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
        // driving in open loop
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final Telemetry logger = new Telemetry(MaxSpeed);
        Autos auton;

        public RobotContainer() {
                // intakeSubsystem.init();
                // climberSubsystem.init();

                auton = new Autos(drivetrain);
                configureBindings();
        }

        private void configureBindings() {

                // Drive
                drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-commandXboxController.getLeftY() * MaxSpeed) // Drive
                                                                                                             // forward
                                                                                                             // with
                                                // negative Y (forward)
                                                .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed) // Drive
                                                                                                             // left
                                                                                                             // with
                                                                                                             // negative
                                                                                                             // X (left)
                                                .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate) // Drive
                                                                                                                         // counterclockwise
                                                                                                                         // with
                                                                                                                         // negative
                                                                                                                         // X
                                                                                                                         // (left)
                                ));
                // reset the field-centric heading by pressing start button/hamburger menu
                // button
                commandXboxController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

                commandXboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                commandXboxController.b().whileTrue(drivetrain
                                .applyRequest(() -> point
                                                .withModuleDirection(new Rotation2d(-commandXboxController.getLeftY(),
                                                                -commandXboxController.getLeftX()))));

                commandXboxController.povUp().onTrue(new InstantCommand(() -> arm.rotateBack()));
                commandXboxController.povRight().onTrue(new InstantCommand(() -> arm.rotateFront()));
                commandXboxController.povDown().onTrue(new InstantCommand(() -> arm.rotateStable()));
                commandXboxController.povLeft().onTrue(new InstantCommand(() -> arm.rotateIntake()));

                commandXboxController.y().onTrue(shooterSubsystem.intakeShooter()).onFalse(shooterSubsystem.stopShooter());
                commandXboxController.leftBumper().onTrue(shooterSubsystem.setShooterSpeed(0.2)).onFalse(shooterSubsystem.setShooterSpeed(0));
                commandXboxController.rightBumper().onTrue(shooterSubsystem.setShooterSpeed(0.85)).onFalse(shooterSubsystem.setShooterSpeed(0));
                commandXboxController.rightTrigger().onTrue(shooterSubsystem.setIndexerSpeed(0.5)).onFalse(shooterSubsystem.setIndexerSpeed(0));

                commandXboxController.leftTrigger().onTrue(intake.deployIntake())
                                        .onTrue(intake.setIntakeSpeed(.3))
                                        .onTrue(new InstantCommand(() -> arm.rotateIntake()))
                                        .onTrue(shooterSubsystem.intakeShooter())

                                        .onFalse(intake.stowIntake())
                                        .onFalse(intake.setIntakeSpeed(0))
                                        .onFalse(shooterSubsystem.stopShooter())
                                        .onFalse(new InstantCommand(() -> arm.rotateStable()));

                // commandXboxController.y().onTrue(climberSubsystem.climbClimber());

                if (Utils.isSimulation()) {
                        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
                }
                drivetrain.registerTelemetry(logger::telemeterize);

                // Climber
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