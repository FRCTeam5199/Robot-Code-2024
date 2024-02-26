package frc.robot.commands;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class Autos extends Command{
  SwerveDrive swerveDrive;
  AprilTagSubsystem aprilTags = new AprilTagSubsystem();

  SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
  HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(3, .01,0), new PIDConstants( 0.85, .00,0.00), 5, .21, new ReplanningConfig());
  PathConstraints pathConstraints = new PathConstraints(1, 1, 1, 1);
  public SendableChooser<Command> autonChooserRed = new SendableChooser<>();
  public SendableChooser<Command> autonChooserBlue = new SendableChooser<>();

  public SendableChooser<Boolean> side = new SendableChooser<>();

  boolean enableAutoAim;


    public Autos(SwerveDrive swerve, IntakeSubsystem intake, ArmSubsystem arm, ShooterSubsystem shooter){
    this.swerveDrive = swerve;
        AutoBuilder.configureHolonomic(()-> swerveDrive.getPose(), swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds, (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)), pathFollowerConfig, ()-> false, swerveDrive);

      PPHolonomicDriveController.setRotationTargetOverride(this::autoAim);

      NamedCommands.registerCommand("deployIntake", new SequentialCommandGroup(
              arm.isAiming(true),
              arm.setArmSetpoint(50),
              new WaitCommand(0.1),
              intake.deployIntake(),
              new WaitCommand(0.2),
              shooter.setIndexerSpeed(-.4),
              arm.rotateIntake(),
              intake.setIntakeSpeed(0.9).onlyIf(()-> arm.getArmEncoder().getPosition()> 1 || arm.getArmEncoder().getPosition() <3),
              shooter.setintakeShooter(true),
              shooter.setRunShooter(true)))  ;

      NamedCommands.registerCommand("retractIntake", new SequentialCommandGroup(intake.setIntakeSpeed(0),
      arm.setArmSetpoint(65),
      new WaitCommand(0.1),
      intake.deployIntake(),
      new WaitCommand(0.2),
      shooter.setIndexerSpeed(-.4),
      arm.rotateIntake(),
      intake.setIntakeSpeed(0.9).onlyIf(() -> arm.getArmEncoder().getPosition() > 1 || arm.getArmEncoder().getPosition() < 3),
      shooter.setintakeShooter(true),
      shooter.setRunShooter(true)));

      NamedCommands.registerCommand("retractIntake",new SequentialCommandGroup( intake.setIntakeSpeed(-.9),
      arm.setArmSetpoint(50),
      new WaitCommand(0.2),
      shooter.setintakeShooter(false),
      shooter.setRunShooter(false),
      intake.stowIntake(),
      shooter.setIndexerSpeed(-0.1),
      new WaitCommand(0.3),
      arm.rotateStable(),
      new WaitCommand(0.5),
      arm.isAiming(false),
      shooter.setIndexerSpeed(0),
      intake.setIntakeSpeed(0)));



      NamedCommands.registerCommand("Sx", new SequentialCommandGroup(arm.setArmSetpoint(150), new WaitCommand(0.5), shooter.runAutonShooting(true), new WaitCommand(0.2), arm.setArmSetpoint(45)));
      NamedCommands.registerCommand("backShot", new SequentialCommandGroup(arm.setArmSetpoint(160), new WaitCommand(0.5), shooter.runAutonShooting(false), new WaitCommand(0.2), arm.setArmSetpoint(45)));
      NamedCommands.registerCommand("topBackShot", new SequentialCommandGroup(arm.setArmSetpoint(170), new WaitCommand(0.8), shooter.runAutonShooting(false), new WaitCommand(0.2)));
      NamedCommands.registerCommand("autoAim", runOnce(()->enableAutoAim = true));
      NamedCommands.registerCommand("shoot", new SequentialCommandGroup(arm.isAutoAiming(true), shooter.setRPMShooter(3000), arm.isAiming(false), shooter.setIndexerSpeed(-.5), new WaitCommand(.5), shooter.setRunIndexer(true)));
      NamedCommands.registerCommand("autoAimOff", runOnce(()->enableAutoAim = false));

      Shuffleboard.getTab("Autons").add("Side", side);
      side.addOption("Red Side", true);
      side.addOption("Blue Side", false);

    Shuffleboard.getTab("Autons").add("Auton Style Red", autonChooserRed).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
    autonChooserRed.addOption("Do nothing", doNothing());

    Shuffleboard.getTab("Autons").add("Auton Style Blue", autonChooserBlue).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
  }
  public Command getAuton() {
    if(DriverStation.getAlliance().get().name() == "Red") {
      System.out.println("Automatically showing autons for Red");
      return autonChooserRed.getSelected();
    } else if (DriverStation.getAlliance().get().name() == "Blue") {
      System.out.println("Automatically showing autons for Blue");
      return autonChooserBlue.getSelected();
    } else if (side.getSelected()) {
      return autonChooserRed.getSelected();
    } else {
      return autonChooserBlue.getSelected();
    }
  }

  public Command onePieceTaxiTopRed(){
    return new PathPlannerAuto("1 Piece Taxi Top Red");
  }
  
    //Autons that don't move
  public Command doNothing(){
        return new WaitCommand(15);
  }

  public Command shootDontMove(){
    return new WaitCommand(15);
  }

  public Optional<Rotation2d> autoAim(){
      if(enableAutoAim){
        return Optional.of(aprilTags.autonAim());
      }else{
        return Optional.empty();
      }
  }


  //Taxi Autons
  public Command taxiTopRed(){
    return new PathPlannerAuto("Taxi Top Red");
  }
    public Command taxiMiddleRed(){
    return new PathPlannerAuto("1 Piece Taxi Middle Red");
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


  public Command onePieceMiddleRed(){
    return new PathPlannerAuto("1 Piece Taxi Middle Red");
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

  public Command twoPieceMiddleRed() {
    return AutoBuilder.buildAuto("2 Piece Middle Red");
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

  public Command threePieceMtBRed() {
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

  public Command fourPieceTopFarRed(){
      return new PathPlannerAuto("4 Piece Top Far Red AUTOAIM");
  }

  //Five Pieces

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

  public Command test(){
      return new PathPlannerAuto("test");
  }

  public Command goToClimb11(){
      return new PathfindHolonomic(
              new Pose2d(11.9047, 3.713, new Rotation2d(60)),
              pathConstraints,
              1,
              swerveDrive::getPose,
              swerveDrive::getCurrentRobotChassisSpeeds,
              (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
              pathFollowerConfig);
  }

  public Command goToClimb12(){
    return new PathfindHolonomic(
            new Pose2d(11.9047, 4.5, new Rotation2d(60)),
            pathConstraints,
            1,
            swerveDrive::getPose,
            swerveDrive::getCurrentRobotChassisSpeeds,
            (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
            pathFollowerConfig);
  }

  public Command goToClimb13(){
    return new PathfindHolonomic(
            new Pose2d(11.22, 4.11, new Rotation2d(60)),
            pathConstraints,
            1,
            swerveDrive::getPose,
            swerveDrive::getCurrentRobotChassisSpeeds,
            (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
            pathFollowerConfig);
  }

  public Command goToClimb14(){
    return new PathfindHolonomic(
            new Pose2d(5.32, 4.11, new Rotation2d(60)),
            pathConstraints,
            1,
            swerveDrive::getPose,
            swerveDrive::getCurrentRobotChassisSpeeds,
            (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
            pathFollowerConfig);
  }

  public Command goToClimb15(){
    return new PathfindHolonomic(
            new Pose2d(4.64, 4.5, new Rotation2d(60)),
            pathConstraints,
            1,
            swerveDrive::getPose,
            swerveDrive::getCurrentRobotChassisSpeeds,
            (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
            pathFollowerConfig);
  }

  public Command goToClimb16(){
    return new PathfindHolonomic(
            new Pose2d(4.64, 3.713, new Rotation2d(60)),
            pathConstraints,
            1,
            swerveDrive::getPose,
            swerveDrive::getCurrentRobotChassisSpeeds,
            (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
            pathFollowerConfig);
  }

  public Command goToAmpRed(){
   return new PathfindHolonomic(
          new Pose2d(14.700757999999999, 7.8742, new Rotation2d(Math.toRadians(90))),
          pathConstraints,
          1,
          swerveDrive::getPose,
          swerveDrive::getCurrentRobotChassisSpeeds,
          (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
          pathFollowerConfig);
  }

   public Command goToAmpBlue(){
    return new PathfindHolonomic(
            new Pose2d(1.8415, 8.2042, new Rotation2d(90)),
            pathConstraints,
            1,
            swerveDrive::getPose,
            swerveDrive::getCurrentRobotChassisSpeeds,
            (speeds)-> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
            pathFollowerConfig);
  }



}