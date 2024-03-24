// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.nio.channels.InterruptedByTimeoutException;
import java.sql.Driver;

import java.sql.Driver;

import javax.swing.text.AbstractDocument.LeafElement;

import org.w3c.dom.CDATASection;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.controls.ButtonPanelButtons;
import frc.robot.utility.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LED.LEDManager;
import frc.robot.commands.Autos;
import frc.robot.constants.MainConstants;
import frc.robot.controls.CommandButtonPanel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystemVer2;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.minor.PivotToCommand;
import frc.robot.subsystems.minor.TagalongPivot;
import frc.robot.utility.superstructure.*;
import frc.robot.utils.TagalongAngle;
import frc.robot.subsystems.minor.ArmPivotSetpoints;
import frc.robot.LED.LEDManager;
// import frc.robot.utility.Akit;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final static AprilTagSubsystem aprilTags = new AprilTagSubsystem();
//     public final static ArmSubsystem arm = new ArmSubsystem();
    public final ArmSubsystemVer2 arm = new ArmSubsystemVer2();
    public final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final static IntakeSubsystem intake = new IntakeSubsystem();
    public final static IndexerSubsystem indexer = new IndexerSubsystem();
    public static TagalongPivot tagAlong;

    public static final CommandButtonPanel buttonPanel = new CommandButtonPanel(MainConstants.OperatorConstants.TOP_BUTTON_PANEL_PORT, MainConstants.OperatorConstants.BOTTOM_BUTTON_PANEL_PORT);
    private final double MaxSpeed = 6; // 8 meters per second desired top speed
    private final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController mainCommandXboxController = new CommandXboxController(
            MainConstants.OperatorConstants.MAIN_CONTROLLER_PORT); // My joystick
    private final CommandXboxController operatorCommandXboxController = new CommandXboxController(
            MainConstants.OperatorConstants.OPERATOR_CONTROLLER_PORT);
        private final CommandXboxController testingController = new CommandXboxController(3);
    private final SwerveDrive drivetrain = TunerConstants.DriveTrain; // My drivetrain
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
    // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    //private final static LEDManager Led = new LEDManager();

    public Command intakeAction;
    public Command stopIntakeAction;
    //     public final static Akit log = new Akit();
    Autos auton;
    
    private PivotToCommand _subArm =
    new PivotToCommand(arm, ArmPivotSetpoints.SUB, true);
    private PivotToCommand _midArm =
    new PivotToCommand(arm, ArmPivotSetpoints.MID, true);
    private PivotToCommand _ampARM =
    new PivotToCommand(arm, ArmPivotSetpoints.AMP, true);
    private PivotToCommand _armStable =
    new PivotToCommand(arm, ArmPivotSetpoints.STABLE, true);
    

    private PivotToCommand _intakeStepUPArm =
    new PivotToCommand(arm, ArmPivotSetpoints.INTAKE_STEP_UP, true);

     private PivotToCommand _backUpArm =
    new PivotToCommand(arm, ArmPivotSetpoints.INTAKE_STEP_UP, true);
    
     private PivotToCommand _upStableArm =
    new PivotToCommand(arm, ArmPivotSetpoints.STABLE, true);


    private PivotToCommand _intakeArm =
    new PivotToCommand(arm, ArmPivotSetpoints.INTAKE, true);
    
    private PivotToCommand _climbMiddleArm =
    new PivotToCommand(arm, ArmPivotSetpoints.CLIMB_MIDDLE, true);
    
    private PivotToCommand _climbMiddleArmV2 =
    new PivotToCommand(arm, ArmPivotSetpoints.CLIMB_MIDDLE, true);
    

    private PivotToCommand _trapArm =
    new PivotToCommand(arm, ArmPivotSetpoints.TRAP, true);
    
    
    private PivotToCommand _customArm =
    new PivotToCommand(arm, ArmPivotSetpoints.ZERO, true);

    
    
    
    private PivotToCommand _vertcicalArm =
    new PivotToCommand(arm, ArmPivotSetpoints.VERTICAL, true);

    

    ConditionalCommand speakerAutoDriveAutoAim = new ConditionalCommand(
            aprilTags.speakerAlignmentRed(),
            aprilTags.speakerAlignmentBlue(),
            () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red
    );
   

    public RobotContainer() {


        tagAlong = arm.getPivot();
        shooterSubsystem.init();
        // arm.init();
        intake.init();
        climberSubsystem.init();
        indexer.init();


        // auton = new Autos(drivetrain, intake, arm, shooterSubsystem, indexer, this);
        // SmartDashboard.putData("Field", drivetrain.m_field);
        configureBindings();
    }

    public void onEnable() {
        arm.onEnable();
    }
    public void onInit(){
        _vertcicalArm.execute();
    }

    public void onDisable() {
        arm.onDisable();
    }
  
    public void disabledPeriodic() {
      arm.disabledPeriodic();
    }

    /**
     * Configures the bindings for commands
     */
    private void configureBindings() {

        intakeAction = new SequentialCommandGroup(
            _intakeStepUPArm.withTimeout(0.1),
            intake.deployIntake(),
            shooterSubsystem.runShooterAtPercent(-.3),
            indexer.setIndexerSpeed(-.4),
            intake.setIntakeSpeed(0.9),
            _intakeArm.withTimeout(0.2)
        );

    
               // stopIntakeAction = new SequentialCommandGroup(
        //     _intakeStepUPArm.withTimeout(0.2),
        //     intake.deployIntake(),
        //     indexer.setIndexerSpeed(-.4),
        //     intake.setIntakeSpeed(0.9),
        //     new WaitCommand(0.2),
        //     _intakeArm.withTimeout(0.2)
        // );

        stopIntakeAction = new SequentialCommandGroup(
                intake.setIntakeSpeed(-.9),
                new WaitCommand(0.2), 
                _backUpArm.withTimeout(0.2),                   
                shooterSubsystem.runShooterAtPercent(0),
                intake.stowIntake(),
                indexer.setIndexerSpeed(-0.1),
                new WaitCommand(0.1),
                indexer.setIndexerSpeed(0),
                intake.setIntakeSpeed(0),
                _upStableArm.withTimeout(0.2));


    //     // new Trigger(() -> indexer.checkForGamePiece()).and(() -> shooterSubsystem.intakeShooter).onTrue(new InstantCommand(() -> mainCommandXboxController.setRumble(1))).onFalse(new InstantCommand(() -> mainCommandXboxController.setRumble(0)));
    //     // new Trigger(() -> shooterSubsystem.reachedSpeed()).onTrue(new InstantCommand(() -> mainCommandXboxController.setRumble(1))).onFalse(new InstantCommand(() -> mainCommandXboxController.setRumble(0)));
    //     // //new Trigger(() -> shooterSubsystem.reachedSpeed()).onTrue(new InstantCommand(() -> Led.green())).onFalse(new InstantCommand(() -> Led.red()));
    //     // // new Trigger(Superstructure::getClimbButtonPressed).onTrue(new frc.robot.utility.DisabledInstantCommand(() -> {
    //     // //         if (DriverStation.isDisabled()) {
    //     // //             ArmSubsystem.toggleBrakeMode();
    //     // //         }
    //     // // }));
    //     // new Trigger(Superstructure::getBrakeButtonPressed).onTrue(new frc.robot.utility.DisabledInstantCommand(() -> {
    //     //     if (DriverStation.isDisabled()) {
    //     //         ArmSubsystemVer2.toggleBrakeMode();
    //     //     }
    //     // }));
    //     new Trigger(Superstructure::getBrakeButtonPressed).onTrue(new frc.robot.utility.DisabledInstantCommand(() -> {
    //         if (DriverStation.isDisabled()) {
    //             ArmSubsystem.toggleBrakeMode();
    //         }
    //     }));

    //     //         // Drive
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                                .withVelocityX(-mainCommandXboxController.getLeftY() * MaxSpeed).withDeadband(0) // Drive
                                // forward
                                // with
                                // negative Y (forward)
                                .withVelocityY(
                                        -mainCommandXboxController.getLeftX() * MaxSpeed).withDeadband(0) // Drive
                                // left
                                // with
                                // negative
                                // X (left)
                                .withRotationalRate(-mainCommandXboxController.getRightX() * MaxAngularRate).withRotationalDeadband(0) // Drive
                        // counterclockwise
                        // with
                        // negative
                        // X
                        // (left)
                        
        ));
        

    //     // // Brake drive
        mainCommandXboxController.button(7).whileTrue(drivetrain.applyRequest(() -> brake));
        // Reorient drive
        mainCommandXboxController.button(8).onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

        mainCommandXboxController.rightBumper().onTrue(indexer.setIndexerSpeed(.4)).onFalse(indexer.setIndexerSpeed(0));
        
    //     mainCommandXboxController.leftBumper().onTrue(new SequentialCommandGroup(arm.isAutoAiming(true),shooterSubsystem.autoAim(), speakerAutoDriveAutoAim)).onFalse(new SequentialCommandGroup(arm.isAutoAiming(false), shooterSubsystem.runShooterAtPercent(0)));
        
        mainCommandXboxController.povLeft().onTrue(new SequentialCommandGroup(shooterSubsystem.runShooterAtRpm(6000))).onFalse(shooterSubsystem.runShooterAtPercent(0));
       
    //     mainCommandXboxController.povDown().onTrue(indexer.setIndexerSpeed(-.1));

    //     mainCommandXboxController.povRight().onTrue(intake.setIntakeSpeed(-1)
    //     ).onFalse(intake.setIntakeSpeed(0));
    //     //climb practice


        mainCommandXboxController.leftTrigger().whileTrue(new InstantCommand(() -> shooterSubsystem.idleShooting = false).andThen(indexer.setIndexerSpeed(-.1).andThen(
                new ConditionalCommand(

                        // climb / amp
                        new ConditionalCommand(
                                shooterSubsystem.runShooterClimbAmp(3300).andThen(indexer.extendServo()),
                                _ampARM.alongWith(shooterSubsystem.runShooterAtRpm(2200).andThen(indexer.extendServo())),
                                () -> climberSubsystem.climbModeEnabled),

                        // normal aiming / auto aiming
                        new SequentialCommandGroup(new InstantCommand(()-> _customArm.changeSetpoint(arm.getSetPoint())),_customArm.alongWith(shooterSubsystem.runShooterPredeterminedRPM())).onlyIf(() -> !shooterSubsystem.intakeShooter),
                                
        //                 // based on climbing on or of
                        () -> shooterSubsystem.ampMode)))).onFalse(
                                shooterSubsystem.runShooterAtPercent(0).andThen(indexer.retractServo()).andThen(_armStable.onlyIf(()->climberSubsystem.climbModeEnabled == false)));

    

        mainCommandXboxController.rightTrigger().whileTrue(intakeAction).onFalse(stopIntakeAction);

        mainCommandXboxController.a().onTrue(shooterSubsystem.setAmpMode(true).andThen(climberSubsystem.setClimbMode(false)));
        mainCommandXboxController.b().onTrue(arm.setSetpoint(ArmPivotSetpoints.SUB.getDegrees()).andThen(shooterSubsystem.setAmpMode(false).andThen(shooterSubsystem.setRPMShooter(4000))));

    //     // drivetrain.registerTelemetry(logger::telemeterize);

        buttonPanel.button(ButtonPanelButtons.ARM_SUB_SETPOINT).onTrue(arm.setSetpoint(ArmPivotSetpoints.SUB.getDegrees()).andThen(shooterSubsystem.setAmpMode(false).andThen(shooterSubsystem.setRPMShooter(4000))));
        buttonPanel.button(ButtonPanelButtons.ARM_BACK_SETPOINT).onTrue(arm.setSetpoint(ArmPivotSetpoints.BACK.getDegrees()).andThen(shooterSubsystem.setAmpMode(false).andThen(shooterSubsystem.setRPMShooter(5000))));
        buttonPanel.button(ButtonPanelButtons.ARM_SAFE_SETPOINT).onTrue(arm.setSetpoint(ArmPivotSetpoints.MID.getDegrees()).andThen(shooterSubsystem.setAmpMode(false).andThen(shooterSubsystem.setRPMShooter(5000))));
        buttonPanel.button(ButtonPanelButtons.ARM_AMP_SETPOINT).onTrue(shooterSubsystem.setAmpMode(true).andThen(climberSubsystem.setClimbMode(false)));
    //     buttonPanel.button(ButtonPanelButtons.ARM_FAR_SHOT_SETPOINT).onTrue(arm.rotateFarShot().andThen(shooterSubsystem.setRPMShooter(6000)));
        // buttonPanel.button(ButtonPanelButtons.ARM_HP_STATION_SETPOINT).onTrue(new SequentialCommandGroup(arm.isAiming(true), arm.rotateHPStation(), shooterSubsystem.runShooterAtPercent(-.4), indexer.setIndexerSpeed(-.2))).onFalse(new SequentialCommandGroup(arm.rotateStable(), shooterSubsystem.runShooterAtPercent(0), indexer.setIndexerSpeed(0), arm.isAiming(false)));
        buttonPanel.button(ButtonPanelButtons.CLIMB_ARM_TRAP_PREP_SETPOINT).onTrue(shooterSubsystem.setAmpMode(true).andThen(shooterSubsystem.setRPMShooter(3300)).andThen(climberSubsystem.setClimbMode(true)).andThen(_climbMiddleArmV2));
        buttonPanel.button(ButtonPanelButtons.CLIMB_ARM_TRAP_SETPOINT).onTrue(shooterSubsystem.setRPMShooter(2500).andThen(shooterSubsystem.setAmpMode(true)).andThen(shooterSubsystem.setRPMShooter(3300)).andThen(climberSubsystem.setClimbMode(true).andThen(_trapArm)));
        // buttonPanel.button(ButtonPanelButtons.MOVE_ARM_SETPOINT_UP).onTrue(arm.increaseOffset(1));
        // buttonPanel.button(ButtonPanelButtons.MOVE_ARM_SETPOINT_DOWN).onTrue(arm.decreaseOffset(1));

        buttonPanel.button(ButtonPanelButtons.MOVE_INTAKE_SETPOINT_UP).onTrue(intake.increaseOffset());
        buttonPanel.button(ButtonPanelButtons.MOVE_INTAKE_SETPOINT_DOWN).onTrue(intake.decreaseOffset());

        buttonPanel.button(ButtonPanelButtons.INCREASE_SHOOTER_SPEED).onTrue(shooterSubsystem.increaseShooterSpeed());
        buttonPanel.button(ButtonPanelButtons.DECREASE_SHOOTER_SPEED).onTrue(shooterSubsystem.decreaseShooterSpeed());

        buttonPanel.button(ButtonPanelButtons.CENTER_CLIMB_SETUP).onTrue(new SequentialCommandGroup(climberSubsystem.setClimbMode(true), shooterSubsystem.setAmpMode(true),intake.deployIntake(), Commands.print("thoough heaven and heart i alone am the honered one"), _climbMiddleArm));
    //     // buttonPanel.button(ButtonPanelButtons.FLIPPY_DO_UP).onTrue(climberSubsystem.setClimbMode(true).andThen(shooterSubsystem.setAmpMode(false)).andThen(arm.isAiming(true)).andThen(shooterSubsystem.setRPMShooter(2100)));
    //     // buttonPanel.button(ButtonPanelButtons.FLIPPY_DO_DOWN).onTrue(climberSubsystem.setClimbMode(false).andThen(shooterSubsystem.setAmpMode(false)).andThen(arm.isAiming(false).andThen(arm.setClimbMode(false))));
        buttonPanel.button(ButtonPanelButtons.CLIMB_UP).whileTrue(climberSubsystem.setClimberSpeed(.8)).onFalse(climberSubsystem.setClimberSpeed(0));
        buttonPanel.button(ButtonPanelButtons.CLIMB_DOWN).whileTrue(climberSubsystem.setClimberSpeed(-.8)).onFalse(climberSubsystem.setClimberSpeed(0));
        buttonPanel.button(ButtonPanelButtons.LEFT_CLIMB_UP).whileTrue(climberSubsystem.setClimberMotor1Speed(.8)).onFalse(climberSubsystem.setClimberMotor1Speed(0));
        buttonPanel.button(ButtonPanelButtons.LEFT_CLIMB_DOWN).whileTrue(climberSubsystem.setClimberMotor1Speed(-.8)).onFalse(climberSubsystem.setClimberMotor1Speed(0));
        buttonPanel.button(ButtonPanelButtons.RIGHT_CLIMB_UP).whileTrue(climberSubsystem.setClimberMotor2Speed(.8)).onFalse(climberSubsystem.setClimberMotor2Speed(0));
        buttonPanel.button(ButtonPanelButtons.RIGHT_CLIMB_DOWN).whileTrue(climberSubsystem.setClimberMotor2Speed(-.8)).onFalse(climberSubsystem.setClimberMotor2Speed(0));


        testingController.a().onTrue(_subArm);
        testingController.b().onTrue(_midArm);
        testingController.povLeft().onTrue(new SequentialCommandGroup(shooterSubsystem.runShooterAtRpm(6000))).onFalse(shooterSubsystem.runShooterAtPercent(0));
       
        // testingController.y().onTrue(new InstantCommand(()-> climberSubsystem.climbModeEnabled = true));
        //     testingController.x().onTrue(new InstantCommand(()-> climberSubsystem.climbModeEnabled = false));
        

        

    //     buttonPanel.button(ButtonPanelButtons.INCREASE_SHOOTER_SPEED).onTrue(shooterSubsystem.increaseShooterSpeed());
    //     buttonPanel.button(ButtonPanelButtons.DECREASE_SHOOTER_SPEED).onTrue(shooterSubsystem.decreaseShooterSpeed());

    //     buttonPanel.button(ButtonPanelButtons.LEFT_CLIMB_UP).onTrue(climberSubsystem.setClimberMotor1Speed(0.8)).onFalse(climberSubsystem.setClimberMotor1Speed(0));
    //     buttonPanel.button(ButtonPanelButtons.LEFT_CLIMB_DOWN).onTrue(climberSubsystem.setClimberMotor1Speed(-0.8)).onFalse(climberSubsystem.setClimberMotor1Speed(0));
    //     buttonPanel.button(ButtonPanelButtons.RIGHT_CLIMB_UP).onTrue(climberSubsystem.setClimberMotor2Speed(0.8)).onFalse(climberSubsystem.setClimberMotor2Speed(0));
    //     buttonPanel.button(ButtonPanelButtons.RIGHT_CLIMB_DOWN).onTrue(climberSubsystem.setClimberMotor2Speed(-0.8)).onFalse(climberSubsystem.setClimberMotor2Speed(0));
    //     buttonPanel.button(ButtonPanelButtons.CLIMB_UP).onTrue(climberSubsystem.setClimberSpeed(0.8)).onFalse(climberSubsystem.setClimberSpeed(0));
    //     buttonPanel.button(ButtonPanelButtons.CLIMB_DOWN).onTrue(climberSubsystem.setClimberSpeed(-0.8)).onFalse(climberSubsystem.setClimberSpeed(0));

    //     buttonPanel.button(ButtonPanelButtons.AUX_RIGHT_TOP).onTrue(intake.deployIntake());
    //     buttonPanel.button(ButtonPanelButtons.AUX_RIGHT_BOTTOM).onTrue(intake.stowIntake());
    //     buttonPanel.button(ButtonPanelButtons.AUX_LEFT_TOP).onTrue(shooterSubsystem.setRunShooter(true).andThen(shooterSubsystem.setintakeShooter(true))).onFalse(shooterSubsystem.setRunShooter(false).andThen(shooterSubsystem.setintakeShooter(false)));
    // }

    // public Command runAutoShooting() {
    //     return new SequentialCommandGroup(shooterSubsystem.runShooterAtPercent(.60), new WaitCommand(0.35), indexer.setIndexerSpeed(.2),
    //             new WaitCommand(0.2), shooterSubsystem.runShooterAtPercent(0), indexer.setIndexerSpeed(0));
    // }

    // public Command autoFarShot() {
    //     return new SequentialCommandGroup(shooterSubsystem.runShooterAtPercent(1),
    //             new WaitCommand(1),
    //             indexer.setIndexerSpeed(.2),
    //             new WaitCommand(0.3), shooterSubsystem.runShooterAtPercent(0),
    //             indexer.setIndexerSpeed(0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return auton.twoPieceMiddleBlue();
        // return auton.getAuton();
        return new WaitCommand(0.2);

    }
}
