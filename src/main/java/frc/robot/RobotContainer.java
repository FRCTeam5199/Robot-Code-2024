         // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.robot.utility.UserInterface.ROBOT_TAB;

import java.time.Instant;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import frc.robot.commands.CompressorCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.controls.ManualControls;
import frc.robot.controls.customcontrollers.CommandButtonPanel;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.piecemanipulation.ArmSubsystem;
import frc.robot.subsystems.piecemanipulation.ClawSubsystem;
import frc.robot.subsystems.piecemanipulation.ElevatorSubsystem;
import frc.robot.subsystems.piecemanipulation.IntakeSubsystem;
import frc.robot.subsystems.piecemanipulation.WristSubsystem;
import frc.robot.utility.AlwaysRunCommand;
import frc.robot.utility.LimelightManager;
import frc.robot.utility.UserInterface;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {


  // public CommandXboxController commandXboxController;
  public CommandButtonPanel buttonPanel;

double MaxSpeed = 6;
double MaxAngularRate = 2;
  CommandXboxController mainCommandXboxController = new CommandXboxController(0);
  public UserInterface uI;

    private final ManualControls manualControls = new ManualControls(new XboxController(0));


  private final SwerveDrive drivetrain = frc.robot.constants.TunerConstants.DriveTrain;

  // not public or private so Robot.java has access to it.
  public final static ArmSubsystem arm = new ArmSubsystem();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  


  public static ElevatorSubsystem elevator = new ElevatorSubsystem();

  public static WristSubsystem wrist = new WristSubsystem();

  public static final ClawSubsystem claw = new ClawSubsystem();

  public static final IntakeSubsystem intake = new IntakeSubsystem();

  public final CompressorSubsystem compressor = new CompressorSubsystem();



  SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    

    compressor.init();

    claw.init();

    elevator.init();

    arm.init();

    wrist.init();

    intake.init();




    createControllers();

    configureBindings();

    CompressorCommand compressorRun = new CompressorCommand(compressor);
    
    compressor.setDefaultCommand(compressorRun);

    
      ROBOT_TAB.add(autonChooser);
  }

  private void createControllers() {
    buttonPanel = new CommandButtonPanel();

  }

  public void configureBindings() {
    // Stable command composition
    ConditionalCommand stableCommandGroup = 
      new ConditionalCommand(
          new ParallelCommandGroup(
            new InstantCommand(() -> elevator.low()),
            new InstantCommand(() -> arm.retract()),
            new InstantCommand(() -> arm.rotateStable())
          ),
          new ParallelCommandGroup(
            new SequentialCommandGroup(
              new InstantCommand(() -> arm.retract()),
              new InstantCommand(() -> elevator.low()),
              new InstantCommand(() -> arm.rotateStable())
            ),  
            new SequentialCommandGroup(
              new InstantCommand(() -> wrist.moveLeft()),
              new WaitCommand(0.5),
              new InstantCommand(() -> wrist.stopRotation())
            )
          ),
        arm::isFront
      );

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
                        
         ));

    // Human player command composition
    ConditionalCommand humanPlayerCommandGroup = 
      new ConditionalCommand(
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.humanPlayer()),
          new InstantCommand(() -> arm.rotateHumanPlayer()),
          new InstantCommand(() -> arm.extendHumanPlayer())
        ),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> arm.retract()),
            new InstantCommand(() -> elevator.low()),
            new InstantCommand(() -> arm.rotateHumanPlayer()),
            new WaitCommand(0.8),
            new InstantCommand(() -> elevator.humanPlayer()),
            new InstantCommand(() -> arm.extendHumanPlayer())
          ),
          new SequentialCommandGroup(
              new InstantCommand(() -> wrist.moveLeft()),
              new WaitCommand(0.5),
              new InstantCommand(() -> wrist.stopRotation())
          )
        ),
      arm::isFront);

    // High goal command composition
    ConditionalCommand highGoalCommandGroup = 
      new ConditionalCommand(
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> elevator.low()),
            new InstantCommand(() -> arm.retract()),
            new InstantCommand(() -> arm.rotateHigh()),
            new WaitCommand(0.8),
            new InstantCommand(() -> arm.extend()),
            new InstantCommand(() -> elevator.high())
          ),
          new SequentialCommandGroup(
              new InstantCommand(() -> wrist.moveRight()),
              new WaitCommand(0.5),
              new InstantCommand(() -> wrist.stopRotation())
          )
        ),
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.high()),
          new InstantCommand(() -> arm.rotateHigh()),
          new InstantCommand(() -> arm.extend())
        ),
      arm::isFront);

    // Medium goal command composition
    ConditionalCommand midGoalCommandGroup = 
      new ConditionalCommand(
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> arm.retract()),
            new InstantCommand(() -> elevator.low()),
            new InstantCommand(() -> arm.rotateMedium()),
            new WaitCommand(0.8),
            new InstantCommand(() -> arm.extendMedium()),
            new InstantCommand(() -> elevator.medium())
          ),
          new SequentialCommandGroup(
              new InstantCommand(() -> wrist.moveRight()),
              new WaitCommand(0.5),
              new InstantCommand(() -> wrist.stopRotation())
          )
        ),
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.medium()),
          new InstantCommand(() -> arm.rotateMedium()),
          new InstantCommand(() -> arm.extendMedium())
        ),
      arm::isFront);
    // Low goal command composition
    ConditionalCommand lowGoalCommandGroup = 
      new ConditionalCommand(
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> arm.retract()),
            new InstantCommand(() -> elevator.low()),
            new InstantCommand(() -> arm.rotateLow()),
            new WaitCommand(0.8),
            new InstantCommand(() -> arm.extendLow())
          ),
          new SequentialCommandGroup(
              new InstantCommand(() -> wrist.moveRight()),
              new WaitCommand(0.5),
              new InstantCommand(() -> wrist.stopRotation())
          )
        ),
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.low()),
          new InstantCommand(() -> arm.rotateLow()),
          new InstantCommand(() -> arm.extendLow())
        ),
      arm::isFront);
    
      ConditionalCommand hp1CommandGroup =
      new ConditionalCommand(
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.humanPlayer()),
          new InstantCommand(() -> arm.rotateHumanPlayer()),
          new InstantCommand(() -> arm.extendHumanPlayer1())
        ),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> arm.retract()),
            new InstantCommand(() -> elevator.low()),
            new InstantCommand(() -> arm.rotateHumanPlayer()),
            new WaitCommand(0.8),
            new InstantCommand(() -> elevator.humanPlayer()),
            new InstantCommand(() -> arm.extendHumanPlayer1())
          ),
          new SequentialCommandGroup(
              new InstantCommand(() -> wrist.moveLeft()),
              new WaitCommand(0.5),
              new InstantCommand(() -> wrist.stopRotation())
          )
        ),
             arm::isFront);
    // Map position commands to button panel triggers
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_2, 12).onTrue(humanPlayerCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 7).onTrue(stableCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 8).onTrue(stableCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 9).onTrue(highGoalCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 10).onTrue(midGoalCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 11).onTrue(lowGoalCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_2, 11).onTrue(hp1CommandGroup);

    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 3).onTrue(intake.ManualRetract());

    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 5).onTrue(wrist.moveLeftManual());
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 6).onTrue(wrist.moveRigthManual());


    manualControls.leftTrigger().onTrue(new ConditionalCommand(
      new SequentialCommandGroup(new InstantCommand(() -> arm.setHighDunk())),
      new SequentialCommandGroup(new InstantCommand(() -> arm.setMidDunk())),
      arm::isHigh));

    manualControls.leftTrigger().onFalse(new InstantCommand(() -> arm.resetDunk()));


    // Map claw commands toxbox controler triggers
    if (Constants.ENABLE_CLAW) {
      manualControls.y().onTrue(claw.openPiston());
      manualControls.a().onTrue(claw.closePiston());
    }

    // Map claw commands toxbox controler triggers
    if (Constants.ENABLE_INTAKE) {
        // manualControls.b().onTrue(intake.spinOutakeOnBottom(false)).onFalse(intake.spinOutakeOnBottom(true));
        manualControls.x().toggleOnTrue(intake.spinBottomWithLimit());
        manualControls.b().onTrue(intake.fastOutake().andThen(intake.stopSpinToKeep())).onFalse((intake.stopSpin()));
    }
  
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 1).onTrue(arm.changeRotateOffset(1));
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 2).onTrue(arm.changeRotateOffset(-1));
    
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_2, 1).onTrue(arm.changeExention(0.5));
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_2, 10).onTrue(arm.changeExention(-0.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
