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
  HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(.4, .001,0), new PIDConstants(.2, .01,0), 5, .33, new ReplanningConfig());
  public SendableChooser<Command> autonChooserRed = new SendableChooser<>();
  public SendableChooser<Command> autonChooserBlue = new SendableChooser<>();

  public SendableChooser<Boolean> side = new SendableChooser<>();

  private HashMap<String, Command> eventMap;


  public Autos(SwerveDrive swerve){
    this.swerveDrive = swerve;
        AutoBuilder.configureHolonomic(()-> swerve.getPose(), swerve::seedFieldRelative, swerve::getCurrentRobotChassisSpeeds, (speeds)-> swerve.setControl(autonDrive.withSpeeds(speeds)), pathFollowerConfig, ()-> false, swerve);
    //AutoBuilder.configureHolonomic(()-> swerveDrive.getPose(), swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds, (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)), pathFollowerConfig, ()-> false, swerveDrive);
    eventMap = new HashMap<>();

    Shuffleboard.getTab("Autons").add("Side", side);
    side.addOption("Red Side", true);
    side.addOption("Blue Side", false);

    Shuffleboard.getTab("Autons").add("Auton Style Red", autonChooserRed).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
    autonChooserRed.addOption("Do nothing", doNothing());
    autonChooserRed.addOption("Shoot and Taxi Middle", shootTaxiMiddleRed());


    Shuffleboard.getTab("Autons").add("Auton Style Blue", autonChooserBlue).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);




  }
  public Command getAuton(){
    if(side.getSelected()){
      return autonChooserRed.getSelected();
    }else{
      return autonChooserBlue.getSelected();
    }
  }
  
  //Autons that don't move
  public Command doNothing(){
        return new WaitCommand(15);
  }

  public Command shootDontMove(){
    return new WaitCommand(15);
  }


  //Taxi Autons
  public Command taxiTopRed(){
    return new PathPlannerAuto("Taxi Top Red");
  }
    public Command taxiMiddleRed(){
    return new PathPlannerAuto("Taxi Middle Red");
  }
    public Command taxiBottomRed(){
    return new PathPlannerAuto("Taxi Bottom Red");
  }
    public Command taxiTopBlue(){
    return new PathPlannerAuto("Taxi Top Blue");
  }
    public Command taxiMiddleBlue(){
    return new PathPlannerAuto("Taxi Middle Blue");
  }
    public Command taxiBottomBlue(){
    return new PathPlannerAuto("Taxi Bottom Blue");
  }

  //One piece Autons
  public Command shootTaxiTopRed(){
    return new PathPlannerAuto("Shoot and Taxi Top Red");
  }
  public Command shootAndTaxiBottomBlue() {
    return new PathPlannerAuto("Blue Shoot and Taxi Bottom");
  }


  public Command shootTaxiMiddleRed(){
    return new PathPlannerAuto("Shoot and Taxi Middle Red");
  }
  public Command shootTaxiBottomRed(){
    return new PathPlannerAuto("Shoot and Taxi Bottom Red");
  }

  public Command shootTaxiTopBlue(){
    return new PathPlannerAuto("Shoot and Taxi Top Blue");
  }
    public Command shootTaxiMiddleBlue(){
    return new PathPlannerAuto("Shoot and Taxi Middle Blue");
  }
    public Command shootTaxiBottomBlue(){
    return new PathPlannerAuto("Shoot and Taxi Bottom Blue");
  }


  //Two Piece autons
  public Command twoPieceTopRed(){
    return new PathPlannerAuto("2 Piece Top Red");
  }


  public Command twoPieceBottomBlue() {
    return AutoBuilder.buildAuto("2 Piece Bottom Blue");
  }

  public Command twoPieceExtendedRed(){
    return new PathPlannerAuto("2 Piece Center Top Red");
    }

  //Three Piece Autons
  public Command threePieceTtMBlue(){
    return new PathPlannerAuto("3 Piece Top Blue");
  }

  public Command threePieceTtMRed(){
    return new PathPlannerAuto("3 Piece Top Red");
  }

  public Command fourPieceBlue(){
    return new PathPlannerAuto("4 Piece Blue");
  }

<<<<<<< HEAD
  public Command shootTaxiTopRed(){
    return new PathPlannerAuto("Red Shoot and Taxi Top");
  }




  
=======
>>>>>>> main
}