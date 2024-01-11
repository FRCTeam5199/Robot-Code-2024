package frc.robot.commands;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

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
  HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(.2, .0001,0), new PIDConstants(.1, .1,0), 5, .33, new ReplanningConfig());
  public SendableChooser<Command> autonChooser = new SendableChooser<>();
  private HashMap<String, Command> eventMap;


  public Autos(SwerveDrive swerve){
    this.swerveDrive = swerve;
        AutoBuilder.configureHolonomic(()-> swerveDrive.getPose(), swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds, (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)), pathFollowerConfig, ()-> false, swerveDrive);
    eventMap = new HashMap<>();

    Shuffleboard.getTab("Auton").add("Auton Style",autonChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);;


    autonChooser.addOption("Do nothing", doNothing());

  }
  public Command getAuton(){
    return autonChooser.getSelected();
  }

  public Command doNothing(){
        return new WaitCommand(15);

    }
  public Command shootDontMove(){
    return new WaitCommand(15);
  }

  public Command shootTaxiRed(){
    return new PathPlannerAuto("Shoot and Taxi Middle");
  }

  public Command twoPieceExtendedRed(){
    return new PathPlannerAuto("2 Piece Center Top Red");
  }

  public Command threePieceTtMBlue(){
    return new PathPlannerAuto("3 Piece Top Blue");
  }

  public Command threePieceTtMRed(){
    return new PathPlannerAuto("3 Piece Top Red");
  }

  public Command fourPieceBlue(){
    return new PathPlannerAuto("4 Piece Blue");
  }

  public Command test(){
    return new PathPlannerAuto("New Auto");
  }

  public Command shootMiddleRed(){
    return new SequentialCommandGroup(shooter.aim(), shooter.shoot());
  }

  public Command redTaxiTop() {
    return new PathPlannerAuto("Red Taxi Top");
  }

  public Command redTaxiMid() {
    return new PathPlannerAuto("Red Taxi Mid");
  }

  public Command redTaxiBot() {
    return new PathPlannerAuto("Red Taxi Bot");
  }
  
}




  
}