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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utility.BrakeButton;
import frc.robot.utility.superstructure.Superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.run;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


    public final static ArmSubsystem arm = new ArmSubsystem();
    public final static IntakeSubsystem intake = new IntakeSubsystem();
    public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public static final AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
    public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final XboxController driveXboxController = new XboxController(0);
    private final double MaxSpeed = 6; // 6 meters per second desired top speed
    private final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController commandXboxController = new CommandXboxController(MainConstants.OperatorConstants.CONTROLLER_PORT); // My joystick
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
        // climberSubsystem.init();

                auton = new Autos(drivetrain);
                configureBindings();
        }

        /**
         * Configures the bindings for commands
         */
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
                        new Trigger(Superstructure::getBrakeButtonPressed).onTrue(new frc.robot.utility.DisabledInstantCommand(() -> {
                                System.out.println("gamer");
                            if (DriverStation.isDisabled()) {
                                ArmSubsystem.toggleBrakeMode();
                            }
                        }));
                        // reset the field-centric heading by pressing start button/hamburger menu button
                        commandXboxController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

                        commandXboxController.button(6).whileTrue(drivetrain.applyRequest(() -> brake));
                        commandXboxController.button(7).whileTrue(drivetrain
                                        .applyRequest(() -> point
                                                        .withModuleDirection(new Rotation2d(-commandXboxController.getLeftY(),
                                                                        -commandXboxController.getLeftX()))));

                        commandXboxController.povUp().onTrue(new InstantCommand(() -> arm.rotateBack()));
                        commandXboxController.povRight().onTrue(new InstantCommand(() -> arm.rotateFront()));
                        commandXboxController.povDown().onTrue(new InstantCommand(() -> arm.rotateStable()));
                        // commandXboxController.povLeft().onTrue(new InstantCommand(() -> arm.rotateIntake()));
                        
                        commandXboxController.b().onTrue(arm.changeArmSetpoint(0.5));
                        commandXboxController.x().onTrue(arm.changeArmSetpoint(-0.5));
                        commandXboxController.y().onTrue(intake.stowIntake());
                        commandXboxController.a().onTrue(intake.deployIntake());
                        
                        commandXboxController.leftBumper().onTrue(shooterSubsystem.setShooterSpeed(0.2)).onFalse(shooterSubsystem.setShooterSpeed(0));
                        commandXboxController.rightBumper().onTrue(shooterSubsystem.setShooterSpeed(0.85)).onFalse(shooterSubsystem.setShooterSpeed(0));
                        commandXboxController.rightTrigger().onTrue(shooterSubsystem.setIndexerSpeed(0.5)).onFalse(shooterSubsystem.setIndexerSpeed(0));
                
                        commandXboxController.leftTrigger()
                                // intake.deployIntake()

                                                .onTrue(intake.setIntakeSpeed(1))
                                                .onTrue(new InstantCommand(() -> arm.rotateIntake()))
                                                .onTrue(shooterSubsystem.intakeShooter())

                                                // .onFalse(intake.stowIntake())
                                                .onFalse(intake.setIntakeSpeed(0))
                                                .onFalse(shooterSubsystem.stopShooter())
                                                .onFalse(new InstantCommand(() -> arm.rotateStable()));

                        commandXboxController.a().onTrue(climberSubsystem.extendClimber());

                if (Utils.isSimulation()) {
                        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
                }
                drivetrain.registerTelemetry(logger::telemeterize);
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
  