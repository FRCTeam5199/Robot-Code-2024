package frc.robot.commands;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.fasterxml.jackson.databind.util.Named;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class Autos extends Command {
    public static SendableChooser<Command> autonChooserRed = new SendableChooser<>();
    public static SendableChooser<Command> autonChooserBlue = new SendableChooser<>();
    public SendableChooser<Boolean> side = new SendableChooser<>();
    SwerveDrive swerveDrive;
    AprilTagSubsystem aprilTags = new AprilTagSubsystem();
    SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
    HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(3, .01, 0), new PIDConstants(1.7, .06, 0.00), 5, .21, new ReplanningConfig());
    PathConstraints pathConstraints = new PathConstraints(1, 1, 1, 1);
    boolean enableAutoAim;


    public Autos(SwerveDrive swerve, IntakeSubsystem intake, ArmSubsystem arm, ShooterSubsystem shooter) {
        this.swerveDrive = swerve;
        AutoBuilder.configureHolonomic(() -> swerveDrive.getPose(), swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds, (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)), pathFollowerConfig, () -> false, swerveDrive);

        PPHolonomicDriveController.setRotationTargetOverride(this::autoAim);

        NamedCommands.registerCommand("deployIntake", new SequentialCommandGroup(
                arm.isAiming(true),
                arm.setArmSetpoint(50),
                new WaitCommand(0.1),
                intake.deployIntake(),
                new WaitCommand(0.2),
                shooter.setIndexerSpeed(-.4),
                arm.rotateIntake(),
                intake.setIntakeSpeed(0.9).onlyIf(() -> arm.getArmEncoder().getPosition() > 1 || arm.getArmEncoder().getPosition() < 3),
                shooter.runShooterAtPercent(-.4)));

        NamedCommands.registerCommand("retractIntake", new SequentialCommandGroup(intake.setIntakeSpeed(-.9),
                arm.setArmSetpoint(50),
                new WaitCommand(0.2),
                shooter.runShooterAtPercent(0),
                intake.stowIntake(),
                shooter.setIndexerSpeed(-0.1),
                new WaitCommand(0.3),
                arm.rotateStable(),
                new WaitCommand(0.5),
                shooter.setIndexerSpeed(0),
                intake.setIntakeSpeed(0),
                arm.isAiming(false),
                shooter.runShooterAtPercent(.5)));

        

        NamedCommands.registerCommand("backShot", new SequentialCommandGroup(arm.isAiming(true), arm.setArmSetpoint(141), new WaitCommand(0.5), shooter.runAutonShooting(), new WaitCommand(0.2), arm.isAiming(false)));
        NamedCommands.registerCommand("topShot", new SequentialCommandGroup(arm.isAiming(true), arm.setArmSetpoint(61), new WaitCommand(0.2), shooter.runAutonShooting(), new WaitCommand(.2), arm.isAiming(false)));
        NamedCommands.registerCommand("midShot", new SequentialCommandGroup(arm.isAiming(true), arm.setArmSetpoint(63.5), new WaitCommand(0.5), shooter.runAutonShooting(), new WaitCommand(0.2), arm.isAiming(false)));
        NamedCommands.registerCommand("bottomShot", new SequentialCommandGroup(arm.isAiming(true), arm.setArmSetpoint(57.5), new WaitCommand(0.5), shooter.runAutonShooting(), new WaitCommand(0.2), arm.isAiming(false)));
        NamedCommands.registerCommand("bottomFarShot",new SequentialCommandGroup(arm.isAiming(true), arm.setArmSetpoint(45.5), new WaitCommand(0.5), shooter.autoFarShot(), new WaitCommand(0.2), arm.isAiming(false)));

        NamedCommands.registerCommand("autoAim", runOnce(() -> enableAutoAim = true));
        NamedCommands.registerCommand("shoot", new SequentialCommandGroup(arm.isAutoAiming(true), arm.isAiming(false), shooter.runShooterAtPercent(1), new WaitCommand(1), shooter.setIndexerSpeed(.4), new WaitCommand(.5), arm.isAutoAiming(false), arm.isAiming(false), shooter.runShooterAtPercent(0),shooter.setIndexerSpeed(0)));
        NamedCommands.registerCommand("autoAimOff", new SequentialCommandGroup(runOnce(() -> enableAutoAim = false), arm.isAutoAiming(false)));

        Shuffleboard.getTab("Autons").add("Side", side);
        side.addOption("Red Side", true);
        side.addOption("Blue Side", false);

        Shuffleboard.getTab("Autons").add("Auton Style Red", autonChooserRed).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
        autonChooserRed.addOption("doNothing", doNothing());
        autonChooserRed.addOption("move do nothing", new SequentialCommandGroup(arm.isAiming(true), arm.setArmSetpoint(141), new WaitCommand(0.5), shooter.runAutonShooting(), new WaitCommand(0.5), arm.rotateSafe(), shooter.runShooterAtPercent(0), shooter.setIndexerSpeed(0), arm.isAiming(false)));

        autonChooserRed.addOption("onePieceTaxiTopRed", onePieceTaxiTopRed());
        autonChooserRed.addOption("onePieceTaxiMiddleRed", onePieceTaxiMiddleRed());
        autonChooserRed.addOption("onePieceTaxiBottomRed", onePieceTaxiBottomRed());
        autonChooserRed.addOption("oneAndHalfPieceTaxiBottomRed", oneAndHalfPieceTaxiBottomRed());

        autonChooserRed.addOption("twoPieceTopRed", twoPieceTopRed());
        autonChooserRed.addOption("twoPieceMiddleRed", twoPieceMiddleRed());
        autonChooserRed.addOption("twoPieceBottomRed", twoPieceBottomRed());
        autonChooserRed.addOption("twoAndHalfBottomRed", twoAndHalfBottomRed());

        autonChooserRed.addOption("threePieceTtMRed", threePieceTtMRed());
   //     autonChooserRed.addOption("threePieceTtMAutoAimRed", threePieceTtMAutoAimRed());
        autonChooserRed.addOption("threePieceBtMAutoAimRed", threePieceBtMAutoAimRed());
    //    autonChooserRed.addOption("threePieceBottomFarAutoAimRed", threePieceBottomFarAutoAimRed());

        autonChooserRed.addOption("fourPieceTtBAutoAimRed", fourPieceTtBAutoAimRed());
        autonChooserRed.addOption("fourPieceTopFarAutoAimRed", fourPieceTopFarAutoAimRed());

        Shuffleboard.getTab("Autons").add("Auton Style Blue", autonChooserBlue).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
        autonChooserBlue.addOption("doNothing", doNothing());
        autonChooserBlue.addOption("move do nothing",new SequentialCommandGroup(arm.isAiming(true), arm.setArmSetpoint(141), new WaitCommand(0.5), shooter.runAutonShooting(), new WaitCommand(0.5), arm.rotateSafe(), shooter.runShooterAtPercent(0), shooter.setIndexerSpeed(0), arm.isAiming(false)));

        autonChooserBlue.addOption("onePieceTaxiTopBlue", onePieceTaxiTopBlue());
        autonChooserBlue.addOption("onePieceTaxiMiddleBlue", onePieceTaxiMiddleBlue());
        autonChooserBlue.addOption("onePieceTaxiBottomBlue", onePieceTaxiBottomBlue());
        autonChooserBlue.addOption("oneAndHalfPieceTaxiBottomBlue", oneAndHalfPieceTaxiBottomBlue());

        autonChooserBlue.addOption("twoPieceTopBlue", twoPieceTopBlue());
        autonChooserBlue.addOption("twoPieceMiddleBlue", twoPieceMiddleBlue());
        autonChooserBlue.addOption("twoPieceBottomBlue", twoPieceBottomBlue());
        autonChooserBlue.addOption("twoAndHalfBottomBlue", twoAndHalfBottomBlue());

        autonChooserBlue.addOption("threePieceBtMBlue", threePieceBtMBlue());
        autonChooserBlue.addOption("threePieceTtMBlue", threePieceTtMBlue());
      //  autonChooserBlue.addOption("threePieceTtMAutoAimBlue", threePieceTtMAutoAimBlue());
      //  autonChooserBlue.addOption("threePieceBtMAutoAimBlue", threePieceBtMAutoAimBlue());
      //  autonChooserBlue.addOption("threePieceBtMAutoAimBlue", threePieceTopFarAutoAimBlue());
      //  autonChooserBlue.addOption("threePieceBottomFarAutoAimBlue", threePieceBottomFarAutoAimBlue());
      //  autonChooserBlue.addOption("threePieceBottomFar", threePieceBottomFarBlue());

        autonChooserRed.setDefaultOption("doNothing", doNothing());
        autonChooserBlue.setDefaultOption("doNothing", doNothing());
    }

    public Command getAuton() {
        if (autonChooserRed.getSelected() != null && autonChooserBlue.getSelected() != null) {
            if (!autonChooserRed.getSelected().getName().contains("15")) {
                return autonChooserRed.getSelected();
            } else if (!autonChooserBlue.getSelected().getName().contains("15")) {
                return autonChooserBlue.getSelected();
            }
        }
        return null;
    }

    public Optional<Rotation2d> autoAim() {
        if (enableAutoAim) {
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                return Optional.of(aprilTags.autonAimRed());
            }else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
                return Optional.of(aprilTags.autonAimBlue());
            }
            else{
                return Optional.empty();
            }
        } else {
            return Optional.empty();
        }
    }

    //Autons that don't move
    public Command doNothing() {
        return new WaitCommand(15);
    }

    //One piece Autons
    public Command onePieceTaxiTopRed() {
        return AutoBuilder.buildAuto("1 Piece Taxi Top Red");
    }

    public Command onePieceTaxiMiddleRed() {
        return AutoBuilder.buildAuto("1 Piece Taxi Middle Red");
    }

    public Command onePieceTaxiBottomRed() {
        return AutoBuilder.buildAuto("1 Piece Taxi Bottom Red");
    }

    public Command onePieceTaxiTopBlue() {
        return AutoBuilder.buildAuto("1 Piece Taxi Top Blue");
    }

    public Command onePieceTaxiMiddleBlue() {
        return AutoBuilder.buildAuto("1 Piece Taxi Middle Blue");
    }

    public Command onePieceTaxiBottomBlue() {
        return AutoBuilder.buildAuto("1 Piece Taxi Bottom Blue");
    }

    public Command oneAndHalfPieceTaxiBottomRed() {
        return AutoBuilder.buildAuto("1.5 Piece Taxi Bottom Red");
    }

    public Command oneAndHalfPieceTaxiBottomBlue() {
        return AutoBuilder.buildAuto("1.5 Piece Taxi Bottom Blue");
    }


    //Two Piece autons
    public Command twoPieceTopRed() {
        return AutoBuilder.buildAuto("2 Piece Top Red");
    }

    public Command twoPieceMiddleRed() {
        return AutoBuilder.buildAuto("2 Piece Middle Red");
    }

    public Command twoPieceBottomRed() {
        return AutoBuilder.buildAuto("2 Piece Bottom Red");
    }

    public Command twoPieceTopBlue() {
        return AutoBuilder.buildAuto("2 Piece Top Blue");
    }

    public Command twoPieceMiddleBlue() {
        return AutoBuilder.buildAuto("2 Piece Middle Blue");
    }

    public Command twoPieceBottomBlue() {
        return AutoBuilder.buildAuto("2 Piece Bottom Blue");
    }

    public Command twoAndHalfBottomRed() {
        return AutoBuilder.buildAuto("2.5 Piece Bottom Red");
    }

    public Command twoAndHalfBottomBlue() {
        return AutoBuilder.buildAuto("2.5 Piece Bottom Blue");
    }


    //Three Piece Regular Autons
    public Command threePieceTtMRed() {
        return AutoBuilder.buildAuto("3 Piece Top to Middle Red");
    }

    public Command threePieceTtMBlue() {
        return AutoBuilder.buildAuto("3 Piece Top to Middle Blue");
    }

    public Command threePieceBtMBlue() {

        return new PathPlannerAuto("3 Piece Bottom to Middle Blue");
    }

    //Three Piece AutoAim Autons
    public Command threePieceTtMAutoAimRed() {
        return AutoBuilder.buildAuto("3 Piece Top to Middle Red AUTOAIM");
    }

    public Command threePieceBtMAutoAimRed() {
        return AutoBuilder.buildAuto("3 Piece Bottom to Middle Red AUTOAIM");
    }

    public Command threePieceTtMAutoAimBlue() {
        return AutoBuilder.buildAuto("3 Piece Top to Middle Blue AUTOAIM");
    }

    public Command threePieceBtMAutoAimBlue() {
        return AutoBuilder.buildAuto("3 Piece Bottom to Middle Blue AUTOAIM");
    }

    public Command threePieceBottomFarAutoAimRed() {
        return AutoBuilder.buildAuto("3 Piece Bottom Far Red AUTOAIM");
    }

    public Command threePieceTopFarAutoAimBlue() {
        return AutoBuilder.buildAuto("3 Piece Top Far Blue AUTOAIM");
    }

    public Command threePieceBottomFarAutoAimBlue() {
        return AutoBuilder.buildAuto("3 Piece Bottom Far Blue AUTOAIM");
    }

    public Command threePieceBottomFarBlue() {
        return AutoBuilder.buildAuto("3 Piece Bottom Far Blue");
    }

    //Four Piece Autons
    public Command fourPieceTopFarAutoAimRed() {
        return AutoBuilder.buildAuto("4 Piece Top Far Red AUTOAIM");
    }

    public Command fourPieceTtBAutoAimRed() {
        return AutoBuilder.buildAuto("4 Piece Top to Bottom Red AUTOAIM");
    }


    public Command test(){
        return AutoBuilder.buildAuto("test");
    }
    public Command goToClimb11() {
        return new PathfindHolonomic(
                new Pose2d(11.9047, 3.713, new Rotation2d(60)),
                pathConstraints,
                1,
                swerveDrive::getPose,
                swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig);
    }

    public Command goToClimb12() {
        return new PathfindHolonomic(
                new Pose2d(11.9047, 4.5, new Rotation2d(60)),
                pathConstraints,
                1,
                swerveDrive::getPose,
                swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig);
    }

    public Command goToClimb13() {
        return new PathfindHolonomic(
                new Pose2d(11.22, 4.11, new Rotation2d(60)),
                pathConstraints,
                1,
                swerveDrive::getPose,
                swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig);
    }

    public Command goToClimb14() {
        return new PathfindHolonomic(
                new Pose2d(5.32, 4.11, new Rotation2d(60)),
                pathConstraints,
                1,
                swerveDrive::getPose,
                swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig);
    }

    public Command goToClimb15() {
        return new PathfindHolonomic(
                new Pose2d(4.64, 4.5, new Rotation2d(60)),
                pathConstraints,
                1,
                swerveDrive::getPose,
                swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig);
    }

    public Command goToClimb16() {
        return new PathfindHolonomic(
                new Pose2d(4.64, 3.713, new Rotation2d(60)),
                pathConstraints,
                1,
                swerveDrive::getPose,
                swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig);
    }

    public Command goToAmpRed() {
        return new PathfindHolonomic(
                new Pose2d(14.700757999999999, 7.8742, new Rotation2d(Math.toRadians(90))),
                pathConstraints,
                1,
                swerveDrive::getPose,
                swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig);
    }

    public Command goToAmpBlue() {
        return new PathfindHolonomic(
                new Pose2d(1.8415, 8.2042, new Rotation2d(90)),
                pathConstraints,
                1,
                swerveDrive::getPose,
                swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig);
    }


}