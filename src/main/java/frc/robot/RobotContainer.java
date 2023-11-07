// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AbstractConstants;
import frc.robot.Constants.MainConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.commands.AprilTagReading;
import frc.robot.subsystems.UserInterface;
import frc.robot.subsystems.drivetrain.swerveDrive.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  UserInterface userInterface;
  public AbstractConstants constants;
  public String config = userInterface.getConfig();
  public Drive drive;
  public SwerveDrive swerveDrive;
  public double x;
  public double y;
  public double rotate;
  public Autos auton;
  public AprilTagSubsystem aprilTag;
  public AprilTagReading aprilTagReading;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(AbstractConstants.CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    AprilTagSubsystem aprilTag = new AprilTagSubsystem();

    userInterface = new UserInterface();
    swerveDrive = new SwerveDrive();
    x = m_driverController.getLeftX();
    y = m_driverController.getLeftY();
    rotate = m_driverController.getRightX();

    drive = new Drive(x, y, rotate);
    swerveDrive.setDefaultCommand(drive);
    auton = new Autos(swerveDrive);


    switch(config){
      case "Main": constants = new MainConstants();
    }

    AprilTagReading = new AprilTagReading(AprilTagSubsystem);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return auton.auton1();
  }
}
