// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.locks.Condition;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utility.Superstructure;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

        private final XboxController driveXboxController = new XboxController(0);
        private final double MaxSpeed = 6; // 6 meters per second desired top speed
        private final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
        /* Setting up bindings for necessary control of the swerve drive platform */
        private final CommandXboxController mainCommandXboxController = new CommandXboxController(
                        MainConstants.OperatorConstants.MAIN_CONTROLLER_PORT); // My joystick
        private final CommandXboxController operatorCommandXboxController = new CommandXboxController(
                        MainConstants.OperatorConstants.OPERATOR_CONTROLLER_PORT);
        private final SwerveDrive drivetrain = TunerConstants.DriveTrain; // My drivetrain
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
        // driving in open loop
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final Telemetry logger = new Telemetry(MaxSpeed);
        
        public final static ArmSubsystem arm = new ArmSubsystem();
        public final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
        public final static IntakeSubsystem intake = new IntakeSubsystem(arm, shooterSubsystem);
        
        Autos auton;

        public RobotContainer() {
                shooterSubsystem.init();
                arm.init();
                intake.init();
                climberSubsystem.init();

                auton = new Autos(drivetrain, intake, arm, shooterSubsystem);
                
                configureBindings();
        }
        

        /**
         * Configures the bindings for commands
         */
        private void configureBindings() {
                 new Trigger(Superstructure::getClimbButtonPressed).onTrue(new frc.robot.utility.DisabledInstantCommand(() -> {
                            if (DriverStation.isDisabled()) {
                                ArmSubsystem.toggleBrakeMode();
                            }
                        }));


                ConditionalCommand switchB = 
                        new ConditionalCommand(
                        climberSubsystem.setClimberSpeed(0.5), 
                        shooterSubsystem.setShooterSpeed(0.2), 
                        climberSubsystem::getClimbMode);
                ConditionalCommand switchX = 
                        new ConditionalCommand(
                        climberSubsystem.setClimberSpeed(-0.5), 
                        shooterSubsystem.setIndexerSpeed(0.2), 
                        climberSubsystem::getClimbMode);

                // Drive
                drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                        drivetrain.applyRequest(() -> drive
                                .withVelocityX(-mainCommandXboxController.getLeftY() * MaxSpeed).withDeadband(1) // Drive
                                                                                                // forward
                                                                                                // with
                                // negative Y (forward)
                                .withVelocityY(-mainCommandXboxController.getLeftX() * MaxSpeed).withDeadband(1) // Drive
                                                                                                // left
                                                                                                // with
                                                                                                // negative
                                                                                                // X (left)
                                .withRotationalRate(-mainCommandXboxController.getRightX() * MaxAngularRate).withRotationalDeadband(1) // Drive
                                                                                                                // counterclockwise
                                                                                                                // with
                                                                                                                // negative
                                                                                                                // X
                                                                                                                // (left)
                        ));           
                mainCommandXboxController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

                mainCommandXboxController.button(6).whileTrue(drivetrain.applyRequest(() -> brake));

                        // operatorCommandXboxController.povUp().onTrue(new SequentialCommandGroup(climberSubsystem.teleOpMode(), arm.teleOpMode()));
                        // operatorCommandXboxController.povRight().onTrue(new InstantCommand(() -> arm.rotateFront()));
                        // operatorCommandXboxController.povDown().onTrue(new SequentialCommandGroup(climberSubsystem.climbMode(), arm.climbMode()));
                        // operatorCommandXboxController.povLeft().onTrue(new InstantCommand(() -> arm.rotateClimb()));


                        // mainCommandXboxController.povUp().onTrue(shooterSubsystem.intakeShooter()).onFalse(shooterSubsystem.stopShooter());
                        // mainCommandXboxController.povRight().onTrue(shooterSubsystem.setBottomShooterSpeed(0.35)).onFalse(shooterSubsystem.setBottomShooterSpeed(0));
                        
                        // operatorCommandXboxController.y().onTrue(arm.changeArmSetpoint(0.5));
                        // operatorCommandXboxController.a().onTrue(arm.changeArmSetpoint(-0.5));

                SequentialCommandGroup speakerMode = new SequentialCommandGroup(
                        climberSubsystem.teleOpMode(),
                        shooterSubsystem.setAmpandClimbMode(false));

                SequentialCommandGroup ampMode = new SequentialCommandGroup(
                        climberSubsystem.teleOpMode(),
                        shooterSubsystem.setAmpandClimbMode(true));

                SequentialCommandGroup climbMode = new SequentialCommandGroup(
                        climberSubsystem.climbMode(),
                        shooterSubsystem.setAmpandClimbMode(true));

                        
                mainCommandXboxController.x().onTrue(speakerMode);
                mainCommandXboxController.b().onTrue(ampMode);
                mainCommandXboxController.a().onTrue(climberSubsystem.setClimberSpeed(-0.5)).onFalse(climberSubsystem.setClimberSpeed(0));

                //For Debugging
                mainCommandXboxController.y().onTrue(climberSubsystem.setClimberSpeed(0.5)).onFalse(climberSubsystem.setClimberSpeed(0));

                mainCommandXboxController.povDown().onTrue(climbMode);

                mainCommandXboxController.leftBumper().whileTrue(drivetrain
                        .applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(-mainCommandXboxController.getLeftY(),
                                        -mainCommandXboxController.getLeftX()))));
                mainCommandXboxController.rightBumper().onTrue(shooterSubsystem.setRunIndexer(true))
                        .onFalse(shooterSubsystem.setRunIndexer(false));

                mainCommandXboxController.leftTrigger().onTrue(shooterSubsystem.setRunShooter(true))
                        .onFalse(shooterSubsystem.setRunShooter(false));
                mainCommandXboxController.rightTrigger().onTrue(new SequentialCommandGroup(
                                intake.deployIntake(),
                                new WaitCommand(0.3),
                                new InstantCommand(() -> arm.rotateIntake()),
                                new WaitCommand(0.15),
                                intake.setIntakeSpeed(1),
                                shooterSubsystem.setintakeShooter(true),
                                shooterSubsystem.setRunShooter(true),
                                shooterSubsystem.setRunIndexer(true)))
                        .onFalse(new SequentialCommandGroup(
                                intake.setIntakeSpeed(0),
                                new InstantCommand(() -> arm.rotateStable()),
                                new WaitCommand(0.2),
                                shooterSubsystem.setintakeShooter(false),
                                shooterSubsystem.setRunShooter(false),
                                shooterSubsystem.setRunIndexer(false),
                                intake.stowIntake()));

                // mainCommandXboxController.rightTrigger().onTrue(intake.deployIntake());
                // mainCommandXboxController.povRight().onTrue(intake.stowIntake());
                // mainCommandXboxController.povLeft().onTrue(new InstantCommand(() -> arm.rotateIntake()));
                        
                        
                // operatorCommandXboxController.a().onTrue(()-> new InstantCommand(arm.getEncoder()));
                // operatorCommandXboxController.y().onTrue(climberSubsystem.setRighttClimberSpeed(0.5)).onFalse(climberSubsystem.setRighttClimberSpeed(0));
                // operatorCommandXboxController.x().onTrue(climberSubsystem.setLeftClimberSpeed(0.5)).onFalse(climberSubsystem.setLeftClimberSpeed(0));
                // operatorCommandXboxController.b().onTrue(climberSubsystem.setRighttClimberSpeed(-0.5)).onFalse(climberSubsystem.setRighttClimberSpeed(0));
                // operatorCommandXboxController.a().onTrue(climberSubsystem.setLeftClimberSpeed(-0.5)).onFalse(climberSubsystem.setLeftClimberSpeed(0));
                        // operatorCommandXboxController.b().onTrue(switchB).onFalse(climberSubsystem.setClimberSpeed(0).andThen(shooterSubsystem.setShooterSpeed(0)));
                        // operatorCommandXboxController.x().onTrue(switchX).onFalse(climberSubsystem.setClimberSpeed(0).andThen(shooterSubsystem.setIndexerSpeed(0)));

                        // operatorCommandXboxController.a().onTrue(()-> new InstantCommand(arm.getEncoder()));
                        // operatorCommandXboxController.x().onTrue(climberSubsystem.setClimberMotor1Speed(0.5)).onFalse(climberSubsystem.setClimberMotor1Speed(0));
                        // operatorCommandXboxController.b().onTrue(climberSubsystem.setClimberMotor1Speed(-0.5)).onFalse(climberSubsystem.setClimberMotor1Speed(0));
                        // operatorCommandXboxController.a().onTrue(climberSubsystem.setClimberMotor2Speed(0.5)).onFalse(climberSubsystem.setClimberMotor2Speed(0));
                        // operatorCommandXboxController.y().onTrue(climberSubsystem.setClimberMotor2Speed(-0.5)).onFalse(climberSubsystem.setClimberMotor2Speed(0));

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
        return null;//auton.twoPieceExtendedRed();
    }
}
