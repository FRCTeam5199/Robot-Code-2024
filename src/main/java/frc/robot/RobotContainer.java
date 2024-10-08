// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.utility.UserInterface.ROBOT_TAB;

import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.piecemanipulation.ArmSubsystem;
import frc.robot.subsystems.piecemanipulation.ClawSubsystem;
import frc.robot.subsystems.piecemanipulation.ElevatorSubsystem;
import frc.robot.subsystems.piecemanipulation.IntakeSubsystem;
import frc.robot.subsystems.piecemanipulation.WristSubsystem;
import frc.robot.utility.UserInterface;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController commandXboxController = new CommandXboxController(0);

  public UserInterface UI;

  // not public or private so Robot.java has access to it.
  public final static ArmSubsystem arm = new ArmSubsystem();

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

    configureBindings();

    ROBOT_TAB.add(autonChooser);
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
    commandXboxController.rightBumper().onTrue(humanPlayerCommandGroup);
    commandXboxController.leftBumper().onTrue(hp1CommandGroup);

    commandXboxController.povRight().onTrue(stableCommandGroup);

//     commandXboxController.rightTrigger().onTrue(intake.ManualRetract());
    commandXboxController.leftTrigger().onTrue(new ConditionalCommand(
      new SequentialCommandGroup(new InstantCommand(() -> arm.setHighDunk())),
      new SequentialCommandGroup(new InstantCommand(() -> arm.setMidDunk())),
      arm::isHigh)).onFalse(new InstantCommand(() -> arm.resetDunk()));

    commandXboxController.povUp().onTrue(highGoalCommandGroup);
    commandXboxController.povLeft().onTrue(midGoalCommandGroup);
    commandXboxController.povDown().onTrue(lowGoalCommandGroup);

    commandXboxController.button(7).onTrue(wrist.moveRightManual());
    commandXboxController.button(8).onTrue(wrist.moveLeftManual());

    // Map claw commands toxbox controller triggers
    if (Constants.ENABLE_CLAW) {
      commandXboxController.y().onTrue(claw.openPiston());
      commandXboxController.a().onTrue(claw.closePiston());
    }

    if (Constants.ENABLE_INTAKE) {
        commandXboxController.x().toggleOnTrue(intake.spinBottomWithLimit());
        commandXboxController.b().onTrue(intake.fastOutake().andThen(intake.stopSpinToKeep())).onFalse((intake.stopSpin()));
    }
  
    // buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 1).onTrue(arm.changeRotateOffset(1));
    // buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 2).onTrue(arm.changeRotateOffset(-1));
    
    // buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_2, 1).onTrue(arm.changeExention(0.5));
    // buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_2, 10).onTrue(arm.changeExention(-0.5));
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
