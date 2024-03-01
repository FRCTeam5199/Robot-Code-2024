// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.controls.ButtonPanelButtons;
import frc.robot.utility.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.constants.MainConstants;
import frc.robot.controls.CommandButtonPanel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utility.superstructure.*;
// import frc.robot.utility.Akit;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final static AprilTagSubsystem aprilTags = new AprilTagSubsystem();
    public final static ArmSubsystem arm = new ArmSubsystem();
    public final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final static IntakeSubsystem intake = new IntakeSubsystem();


    public static final CommandButtonPanel buttonPanel = new CommandButtonPanel(MainConstants.OperatorConstants.TOP_BUTTON_PANEL_PORT, MainConstants.OperatorConstants.BOTTOM_BUTTON_PANEL_PORT);
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

    public Command intakeAction;
    public Command stopIntakeAction;
    //     public final static Akit log = new Akit();
    Autos auton;


    public RobotContainer() {
        shooterSubsystem.init();
        arm.init();
        intake.init();
        climberSubsystem.init();


        auton = new Autos(drivetrain, intake, arm, shooterSubsystem);
        // SmartDashboard.putData("Field", drivetrain.m_field);
        configureBindings();
    }

    /**
     * Configures the bindings for commands
     */
    private void configureBindings() {
        ConditionalCommand speakerAutoDriveAutoAim = new ConditionalCommand(
                aprilTags.speakerAlignmentRed(),
                aprilTags.speakerAlignementBlue(),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        );


        intakeAction = new SequentialCommandGroup(
                arm.isAiming(true),
                arm.setArmSetpoint(50),
                new WaitCommand(0.1),
                intake.deployIntake(),
                new WaitCommand(0.2),
                shooterSubsystem.setIndexerSpeed(-.4),
                arm.rotateIntake(),
                intake.setIntakeSpeed(0.9).onlyIf(() -> arm.getArmEncoder().getPosition() > 1 || arm.getArmEncoder().getPosition() < 3),
                shooterSubsystem.runShooterAtPercent(-.6));


        stopIntakeAction = new SequentialCommandGroup(
                intake.setIntakeSpeed(-.9),
                arm.setArmSetpoint(50),
                new WaitCommand(0.2),
                shooterSubsystem.runShooterAtPercent(0),
                intake.stowIntake(),
                shooterSubsystem.setIndexerSpeed(-0.1),
                new WaitCommand(0.3),
                arm.rotateStable(),
                new WaitCommand(0.5),
                arm.isAiming(false),
                shooterSubsystem.setIndexerSpeed(0),
                intake.setIntakeSpeed(0));


        new Trigger(() -> shooterSubsystem.checkForGamePiece()).and(() -> shooterSubsystem.intakeShooter).onTrue(new InstantCommand(() -> mainCommandXboxController.setRumble(1))).onFalse(new InstantCommand(() -> mainCommandXboxController.setRumble(0)));
        new Trigger(() -> shooterSubsystem.reachedSpeed()).onTrue(new InstantCommand(() -> mainCommandXboxController.setRumble(1))).onFalse(new InstantCommand(() -> mainCommandXboxController.setRumble(0)));

        // new Trigger(Superstructure::getClimbButtonPressed).onTrue(new frc.robot.utility.DisabledInstantCommand(() -> {
        //         if (DriverStation.isDisabled()) {
        //             ArmSubsystem.toggleBrakeMode();
        //         }
        // }));
        new Trigger(Superstructure::getBrakeButtonPressed).onTrue(new frc.robot.utility.DisabledInstantCommand(() -> {
            if (DriverStation.isDisabled()) {
                ArmSubsystem.toggleBrakeMode();
            }
        }));

        //         // Drive
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                                .withVelocityX(-mainCommandXboxController.getLeftY() * MaxSpeed).withDeadband(1) // Drive
                                // forward
                                // with
                                // negative Y (forward)
                                .withVelocityY(
                                        -mainCommandXboxController.getLeftX() * MaxSpeed).withDeadband(1) // Drive
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

        // Brake drive
        mainCommandXboxController.button(7).whileTrue(drivetrain.applyRequest(() -> brake));
        // Reorient drive
        mainCommandXboxController.button(8).onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

        mainCommandXboxController.rightBumper().onTrue(shooterSubsystem.setIndexerSpeed(.4)).onFalse(shooterSubsystem.setIndexerSpeed(0));

        mainCommandXboxController.leftBumper().onTrue(new SequentialCommandGroup(arm.isAutoAiming(true), speakerAutoDriveAutoAim, arm.isAiming(false).andThen(() -> shooterSubsystem.ampMode = false))).onFalse(arm.isAutoAiming(false));


        mainCommandXboxController.povUp().onTrue(arm.isAiming(true).andThen(arm.rotateAmp()).andThen(auton.goToAmpRed())).onFalse(arm.isAiming(false));
        mainCommandXboxController.povDown().onTrue(shooterSubsystem.setIndexerSpeed(-.1));
        mainCommandXboxController.povRight().onTrue(intake.setIntakeSpeed(-1)
        ).onFalse(intake.setIntakeSpeed(0));
        //climb practice
        mainCommandXboxController.leftTrigger().whileTrue(new InstantCommand(() -> shooterSubsystem.idleShooting = false).andThen(shooterSubsystem.setIndexerSpeed(-.1).andThen(
                new ConditionalCommand(

                        // climb / amp
                        new ConditionalCommand(
                                shooterSubsystem.runShooterClimbAmp(3300),
                                arm.isAiming(true).andThen(shooterSubsystem.runShooterClimbAmp(5000)),
                                () -> climberSubsystem.climbModeEnabled),

                        // normal aiming / auto aiming
                        new ConditionalCommand(
                                new SequentialCommandGroup(new InstantCommand(() -> shooterSubsystem.autoTargeting = true), shooterSubsystem.runShooterPredeterminedRPM(), new InstantCommand(() -> shooterSubsystem.idleShooting = false)),
                                new SequentialCommandGroup(new InstantCommand(() -> arm.isAiming = true), new InstantCommand(() -> shooterSubsystem.idleShooting = false), shooterSubsystem.runShooterPredeterminedRPM(), new InstantCommand(() -> System.out.println("normal aiming"))).onlyIf(() -> shooterSubsystem.intakeShooter == false),
                                () -> arm.autoAiming == true),

                        // based on climbing on or of
                        () -> shooterSubsystem.ampMode)))).whileFalse(

                                shooterSubsystem.runShooterAtPercent(0).andThen((new InstantCommand(() -> arm.isAiming = false).onlyIf(() -> climberSubsystem.climbModeEnabled == false)).andThen(new InstantCommand(() -> shooterSubsystem.autoTargeting = false).andThen(shooterSubsystem.runShooterAtRpm(2000).onlyIf(()->arm.autoAiming)))));


        mainCommandXboxController.rightTrigger().whileTrue(intakeAction).onFalse(stopIntakeAction);

        drivetrain.registerTelemetry(logger::telemeterize);

        buttonPanel.button(ButtonPanelButtons.ARM_SUB_SETPOINT).onTrue(arm.setClimbMode(false).andThen(arm.rotateSub()).andThen(shooterSubsystem.setAmpMode(false).andThen(shooterSubsystem.setRPMShooter(4000))));
        buttonPanel.button(ButtonPanelButtons.ARM_BACK_SETPOINT).onTrue(arm.setClimbMode(false).andThen(arm.rotateBack()).andThen(shooterSubsystem.setAmpMode(false).andThen(shooterSubsystem.setRPMShooter(5000))));
        buttonPanel.button(ButtonPanelButtons.ARM_SAFE_SETPOINT).onTrue(arm.setClimbMode(false).andThen(arm.rotateSafe()).andThen(shooterSubsystem.setAmpMode(false).andThen(shooterSubsystem.setRPMShooter(5000))));
        buttonPanel.button(ButtonPanelButtons.ARM_AMP_SETPOINT).onTrue(arm.setClimbMode(false).andThen(arm.rotateAmp()).andThen(shooterSubsystem.setAmpMode(true).andThen(climberSubsystem.setClimbMode(false))));
        buttonPanel.button(ButtonPanelButtons.ARM_FAR_SHOT_SETPOINT).onTrue(arm.rotateFarShot().andThen(shooterSubsystem.setRPMShooter(5000)));
        buttonPanel.button(ButtonPanelButtons.ARM_HP_STATION_SETPOINT).onTrue(arm.rotateHPStation());
        buttonPanel.button(ButtonPanelButtons.CLIMB_ARM_TRAP_PREP_SETPOINT).onTrue(arm.setClimbMode(true).andThen(arm.rotatePrepClimbP2()).andThen(shooterSubsystem.setAmpMode(true)).andThen(shooterSubsystem.setRPMShooter(3300)));
        buttonPanel.button(ButtonPanelButtons.CLIMB_ARM_TRAP_SETPOINT).onTrue(arm.setClimbMode(true).andThen(arm.rotateTrap()).andThen(shooterSubsystem.setRPMShooter(3300)));
        buttonPanel.button(ButtonPanelButtons.MOVE_ARM_SETPOINT_UP).onTrue(arm.increaseOffset(1));
        buttonPanel.button(ButtonPanelButtons.MOVE_ARM_SETPOINT_DOWN).onTrue(arm.decreaseOffset(1));

        buttonPanel.button(ButtonPanelButtons.MOVE_INTAKE_SETPOINT_UP).onTrue(intake.increaseOffset());
        buttonPanel.button(ButtonPanelButtons.MOVE_INTAKE_SETPOINT_DOWN).onTrue(intake.decreaseOffset());

        buttonPanel.button(ButtonPanelButtons.MOVE_ARM_SETPOINT_UP).onTrue(arm.increaseOffset(1));
        buttonPanel.button(ButtonPanelButtons.MOVE_ARM_SETPOINT_DOWN).onTrue(arm.decreaseOffset(1));
        buttonPanel.button(ButtonPanelButtons.INCREASE_SHOOTER_SPEED).onTrue(shooterSubsystem.increaseShooterSpeed());
        buttonPanel.button(ButtonPanelButtons.DECREASE_SHOOTER_SPEED).onTrue(shooterSubsystem.decreaseShooterSpeed());

        buttonPanel.button(ButtonPanelButtons.CENTER_CLIMB_SETUP).onTrue(new SequentialCommandGroup(arm.setClimbMode(true), climberSubsystem.setClimbMode(true), arm.rotateTrapPrep(), shooterSubsystem.setAmpMode(true), arm.isAiming(true), intake.deployIntake(), Commands.print("thoough heaven and heart i alone am the honered one")));
        buttonPanel.button(ButtonPanelButtons.FLIPPY_DO_UP).onTrue(climberSubsystem.setClimbMode(true).andThen(shooterSubsystem.setAmpMode(false)).andThen(arm.isAiming(true)).andThen(shooterSubsystem.setRPMShooter(2100)));
        buttonPanel.button(ButtonPanelButtons.FLIPPY_DO_DOWN).onTrue(climberSubsystem.setClimbMode(false).andThen(shooterSubsystem.setAmpMode(false)).andThen(arm.isAiming(false).andThen(arm.setClimbMode(false))));
        buttonPanel.button(ButtonPanelButtons.CLIMB_UP).whileTrue(climberSubsystem.setClimberSpeed(.8)).onFalse(climberSubsystem.setClimberSpeed(0));
        buttonPanel.button(ButtonPanelButtons.CLIMB_DOWN).whileTrue(climberSubsystem.setClimberSpeed(-.8)).onFalse(climberSubsystem.setClimberSpeed(0));
        buttonPanel.button(ButtonPanelButtons.LEFT_CLIMB_UP).whileTrue(climberSubsystem.setClimberMotor1Speed(.8)).onFalse(climberSubsystem.setClimberMotor1Speed(0));
        buttonPanel.button(ButtonPanelButtons.LEFT_CLIMB_DOWN).whileTrue(climberSubsystem.setClimberMotor1Speed(-.8)).onFalse(climberSubsystem.setClimberMotor1Speed(0));
        buttonPanel.button(ButtonPanelButtons.RIGHT_CLIMB_UP).whileTrue(climberSubsystem.setClimberMotor2Speed(.8)).onFalse(climberSubsystem.setClimberMotor2Speed(0));
        buttonPanel.button(ButtonPanelButtons.RIGHT_CLIMB_DOWN).whileTrue(climberSubsystem.setClimberMotor2Speed(-.8)).onFalse(climberSubsystem.setClimberMotor2Speed(0));


        buttonPanel.button(ButtonPanelButtons.INCREASE_SHOOTER_SPEED).onTrue(shooterSubsystem.increaseShooterSpeed());
        buttonPanel.button(ButtonPanelButtons.DECREASE_SHOOTER_SPEED).onTrue(shooterSubsystem.decreaseShooterSpeed());

        buttonPanel.button(ButtonPanelButtons.LEFT_CLIMB_UP).onTrue(climberSubsystem.setClimberMotor1Speed(0.8)).onFalse(climberSubsystem.setClimberMotor1Speed(0));
        buttonPanel.button(ButtonPanelButtons.LEFT_CLIMB_DOWN).onTrue(climberSubsystem.setClimberMotor1Speed(-0.8)).onFalse(climberSubsystem.setClimberMotor1Speed(0));
        buttonPanel.button(ButtonPanelButtons.RIGHT_CLIMB_UP).onTrue(climberSubsystem.setClimberMotor2Speed(0.8)).onFalse(climberSubsystem.setClimberMotor2Speed(0));
        buttonPanel.button(ButtonPanelButtons.RIGHT_CLIMB_DOWN).onTrue(climberSubsystem.setClimberMotor2Speed(-0.8)).onFalse(climberSubsystem.setClimberMotor2Speed(0));
        buttonPanel.button(ButtonPanelButtons.CLIMB_UP).onTrue(climberSubsystem.setClimberSpeed(0.8)).onFalse(climberSubsystem.setClimberSpeed(0));
        buttonPanel.button(ButtonPanelButtons.CLIMB_DOWN).onTrue(climberSubsystem.setClimberSpeed(-0.8)).onFalse(climberSubsystem.setClimberSpeed(0));

        buttonPanel.button(ButtonPanelButtons.AUX_RIGHT_TOP).onTrue(intake.deployIntake());
        buttonPanel.button(ButtonPanelButtons.AUX_RIGHT_BOTTOM).onTrue(intake.stowIntake());
        buttonPanel.button(ButtonPanelButtons.AUX_LEFT_TOP).onTrue(shooterSubsystem.setRunShooter(true).andThen(shooterSubsystem.setintakeShooter(true))).onFalse(shooterSubsystem.setRunShooter(false).andThen(shooterSubsystem.setintakeShooter(false)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return auton.twoPieceMiddleBlue();
        return auton.test();
    }
}
