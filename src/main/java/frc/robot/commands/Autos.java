package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.path.*;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Autos extends Command{
  SwerveDrive swerveDrive;
  SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
  ShooterSubsystem shooter = new ShooterSubsystem();
  HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(.2, .0001,0), new PIDConstants(.1, .1,0), 5, .4064, new ReplanningConfig());
  public SendableChooser<Command> autonChooser = new SendableChooser<>();

  public Autos(SwerveDrive swerve){
    this.swerveDrive = swerve;
    AutoBuilder.configureHolonomic(()->swerve.getState().Pose, swerve::seedFieldRelative, swerve::getCurrentRobotChassisSpeeds, (speeds)-> swerve.setControl(autonDrive.withSpeeds(speeds)), pathFollowerConfig, swerve);

  }

  public Command doNothing(){
        return new WaitCommand(15);

    }
  public Command shootDontMove(){
    return new WaitCommand(15);
  }

  public Command 2PieceExtendedRed(){
    return AutoBuilder.buildAuto("2 Piece Extended Top Red");
  }

  public Command 3PieceTtMBlue(){
    return AutoBuilder.buildAuto("3 Piece Top Blue");
  }

  public Command 3PieceTtMRed(){
    return AutoBuilder.buildAuto("3 Piece Top Red");
  }




  
}