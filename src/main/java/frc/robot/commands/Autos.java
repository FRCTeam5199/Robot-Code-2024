package frc.robot.commands;

import java.util.HashMap;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Autos extends Command{
  SwerveDrive swerveDrive;

  SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
  HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(3, .01,0), new PIDConstants( 1,1.5,0), 5, .21, new ReplanningConfig());
  public SendableChooser<Command> autonChooserRed = new SendableChooser<>();
  public SendableChooser<Command> autonChooserBlue = new SendableChooser<>();

  public SendableChooser<Boolean> side = new SendableChooser<>();


    public Autos(SwerveDrive swerve, IntakeSubsystem intake, ArmSubsystem arm, ShooterSubsystem shooter){
    this.swerveDrive = swerve;
        // AutoBuilder.configureHolonomic(()-> swerveDrive.getPose(), swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds, (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)), pathFollowerConfig, ()-> false, swerveDrive);
        HashMap<String, Command> eventMap = new HashMap<>();
      NamedCommands.registerCommand("deployIntake", new SequentialCommandGroup(
                                                          intake.deployIntake(),
                                                          new WaitCommand(0.3),
                                                          arm.rotateIntake(),
                                                          new WaitCommand(0.15),
                                                          intake.setIntakeSpeed(0.7),
                                                          shooter.setIntakeShooter(true),
                                                          shooter.setRunShooter(true),
                                                          shooter.setRunIndexer(true)));
      NamedCommands.registerCommand("retractIntake", new SequentialCommandGroup(
                                                          intake.setIntakeSpeed(0),
                                                          arm.rotateStable(),
                                                          new WaitCommand(0.2),
                                                          shooter.setIntakeShooter(false),
                                                          shooter.setRunShooter(false),
                                                          shooter.setRunIndexer(false),
                                                          intake.stowIntake()));

      NamedCommands.registerCommand("wait1", new WaitCommand(1));
      NamedCommands.registerCommand("SbackShot", new SequentialCommandGroup(arm.setArmSetpoint(150), new WaitCommand(0.3), shooter.runAutonShooting(true), new WaitCommand(0.2), arm.setArmSetpoint(45)));
      NamedCommands.registerCommand("backShot", new SequentialCommandGroup(arm.setArmSetpoint(146), new WaitCommand(0.3), shooter.runAutonShooting(false), new WaitCommand(0.2), arm.setArmSetpoint(45)));
      NamedCommands.registerCommand("topPieceShot", new SequentialCommandGroup(arm.rotateTopPiece(), new WaitCommand(0.3), shooter.runAutonShooting(false), new WaitCommand(0.2), arm.setArmSetpoint(45)));
      NamedCommands.registerCommand("bottomPieceShot", new SequentialCommandGroup(arm.rotateBottomPiece(), new WaitCommand(0.3), shooter.runAutonShooting(false), new WaitCommand(0.2), arm.setArmSetpoint(45)));
      NamedCommands.registerCommand("middlePieceShot", new SequentialCommandGroup(arm.rotateMiddlePiece(), new WaitCommand(0.3), shooter.runAutonShooting(false), new WaitCommand(0.2), arm.setArmSetpoint(45)));
      // NamedCommands.registerCommand("topBackShot", new SequentialCommandGroup(arm.setArmSetpoint(50), new WaitCommand(0.8), shooter.runAutonShooting(false), new WaitCommand(0.2)));

      // Shuffleboard.getTab("Autons").add("Side", side);
      side.addOption("Red Side", true);
      side.addOption("Blue Side", false);

    Shuffleboard.getTab("Autons").add("Auton Style Red", autonChooserRed).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
    autonChooserRed.addOption("Do nothing", doNothing());

    Shuffleboard.getTab("Autons").add("Auton Style Blue", autonChooserBlue).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
  }
  public Command getAuton() {
    if(DriverStation.getAlliance().get().name() == "Red") {
      // System.out.println("Automatically showing autons for Red");
      return autonChooserRed.getSelected();
    } else if (DriverStation.getAlliance().get().name() == "Blue") {
      // System.out.println("Automatically showing autons for Blue");
      return autonChooserBlue.getSelected();
    } else if (side.getSelected()) {
      return autonChooserRed.getSelected();
    } else {
      return autonChooserBlue.getSelected();
    }
  }

  public SendableChooser<Boolean> getAutonSide() {
    return side;
  }

  public Command test(){
      return new PathPlannerAuto("test");
  }
  public Command twoPieceTaxiTopRed(){
    return new PathPlannerAuto("2 Piece Taxi Top Red");
    // return new PathPlannerAuto("2 Piece Taxi Top Red");
  }
  
    //Autons that don't move
  public Command doNothing(){
        return new WaitCommand(15);
  }

  public Command shootMiddleDontMove(){
    return new SequentialCommandGroup();
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
 

  //One piece Autons
  public Command onePieceBottomRed(){
    return new PathPlannerAuto(getName());
  }

  public Command onePieceMiddleRed(){
    return new PathPlannerAuto("1 Piece Taxi Middle Red");
  }
  
  public Command onePieceTopRed(){
    return new PathPlannerAuto("1 Piece Taxi Top Red");
  }


  //Two Piece autons
  public Command twoPieceTopRed(){
    return new PathPlannerAuto("2 Piece Top Red");
  }
  

  public Command twoPieceBottomBlue() {
    return new PathPlannerAuto("2 Piece Bottom Blue");
  }
  public Command twoPieceMiddleBlue(){
    return new PathPlannerAuto("2 Piece Middle Blue");
  }

  public Command twoPieceMiddleRed() {
    return new PathPlannerAuto("2 Piece Middle Red");
  }

  public Command twoPieceExtendedRed(){
    return new PathPlannerAuto("2 Piece Center Top Red");
  }

  public Command twoPieceBottomRed() {
    return new PathPlannerAuto("2 Piece Bottom Red");
  }

  //Three Piece Autons
  public Command threePieceTtMBlue(){
    return new PathPlannerAuto("3 Piece Top Blue");
  }

  public Command threePieceTtMRed(){
    return new PathPlannerAuto("3 Piece Top to Middle Red");
  }
  public Command threePieceBtMRed(){
    return new PathPlannerAuto("3 Piece Bottom to Middle Red");
  }
  public Command threePieceMtTRed(){
    return new PathPlannerAuto("3 Piece Middle to Top Red");
  }

  public Command  threePieceMtBRed() {
    return new PathPlannerAuto("3 Piece Middle to Bottom Red");
  }

  //Four Piece Autons
  public Command fourPieceBlue(){
    return new PathPlannerAuto("4 Piece Blue");
  }

  public Command fourPieceTtBRed(){
    return new PathPlannerAuto("4 Piece Top to Bottom Red");
  }
  public Command fourPieceBtTRed(){
    return new PathPlannerAuto("4 Piece Bottom to Top Red");
  }

  public Command fivePieceTtBRed(){
    return new PathPlannerAuto("5 Piece Top to Bottom Red");
  }

  public Command fivePieceBtTRed(){
    return new PathPlannerAuto("5 Piece Bottom to Top Red");
  }

  //6 Piece Autons
  public Command sixPieceBtT(){
    return new PathPlannerAuto("6 Piece Bottom to Top Red");
  }

}