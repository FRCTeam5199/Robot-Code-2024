package frc.robot.commands;

import java.util.Optional;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystemVer2;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.minor.ArmPivotSetpoints;
import frc.robot.subsystems.minor.PivotToCommand;

public class Autos extends Command {
    public static SendableChooser<Command> autonChooserRed = new SendableChooser<>();
    public static SendableChooser<Command> autonChooserBlue = new SendableChooser<>();
    public SendableChooser<Boolean> side = new SendableChooser<>();
    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;
    public IndexerSubsystem indexer;
    public ArmSubsystemVer2 arm;
    SwerveDrive swerveDrive;
    AprilTagSubsystem aprilTags = new AprilTagSubsystem();
    SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
    HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(3, .01, 0), new PIDConstants(1.7, .06, 0.00), 5, .21, new ReplanningConfig());
    PathConstraints pathConstraints = new PathConstraints(1, 1, 1, 1);
    boolean enableAutoAim;
    private PivotToCommand _customArm;
    private PivotToCommand _armStable;
    private PivotToCommand _twoPieceArmStableRed;
    private PivotToCommand _twoPieceArmStableBlue;
    private PivotToCommand _threePieceArmStableRed;
    private PivotToCommand _threePieceArmStableBlue;
    private PivotToCommand _intakeArm;
    private PivotToCommand _upStableArm;
    private PivotToCommand _intakeStepUPArm;
    private PivotToCommand _backUpArm;
    private PivotToCommand _halfIntakeArm;
    private PivotToCommand _halfUpStableArm;
    private PivotToCommand _backShot;
    private PivotToCommand _halfBackShot;
    private PivotToCommand _halfIntakeBackShot;
    private PivotToCommand _twoPieceExtendedShotRed;
    private PivotToCommand _twoPieceExtendedShotBlue;
    private PivotToCommand _threePieceExtendedShotRed;
    private PivotToCommand _threePieceExtendedShotBlue;

    public Autos(SwerveDrive swerve, IntakeSubsystem intake, ArmSubsystemVer2 arm, ShooterSubsystem shooter, IndexerSubsystem indexer, RobotContainer robotContainer) {

        this.shooter = shooter;
        this.intake = intake;
        this.indexer = indexer;
        this.arm = arm;

        _customArm = new PivotToCommand(arm, ArmPivotSetpoints.ZERO, true);
        _armStable = new PivotToCommand(arm, ArmPivotSetpoints.STABLE, true);
        _twoPieceArmStableRed = new PivotToCommand(arm, ArmPivotSetpoints.STABLE, true);
        _twoPieceArmStableBlue = new PivotToCommand(arm, ArmPivotSetpoints.STABLE, true);
        _threePieceArmStableRed = new PivotToCommand(arm, ArmPivotSetpoints.STABLE, true);
        _threePieceArmStableBlue = new PivotToCommand(arm, ArmPivotSetpoints.STABLE, true);
        _intakeArm = new PivotToCommand(arm, ArmPivotSetpoints.INTAKE, true);
        _upStableArm = new PivotToCommand(arm, ArmPivotSetpoints.STABLE, true);
        _intakeStepUPArm = new PivotToCommand(arm, ArmPivotSetpoints.INTAKE_STEP_UP, true);
        _backUpArm = new PivotToCommand(arm, ArmPivotSetpoints.INTAKE_STEP_UP, true);
        _halfIntakeArm = new PivotToCommand(arm, ArmPivotSetpoints.INTAKE, true);
        _halfUpStableArm = new PivotToCommand(arm, ArmPivotSetpoints.STABLE, true);
        _backShot = new PivotToCommand(arm, ArmPivotSetpoints.BACK, true);
        _halfBackShot = new PivotToCommand(arm, ArmPivotSetpoints.BACK, true);
        _halfIntakeBackShot = new PivotToCommand(arm, ArmPivotSetpoints.BACK, true);
        _twoPieceExtendedShotRed = new PivotToCommand(arm, ArmPivotSetpoints.TWO_PIECE_EXTENDED_RED, true);
        _twoPieceExtendedShotBlue = new PivotToCommand(arm, ArmPivotSetpoints.TWO_PIECE_EXTENDED_BLUE, true);
        _threePieceExtendedShotRed = new PivotToCommand(arm, ArmPivotSetpoints.TWO_PIECE_EXTENDED_RED, true);
        _threePieceExtendedShotBlue = new PivotToCommand(arm, ArmPivotSetpoints.TWO_PIECE_EXTENDED_BLUE, true);

        this.swerveDrive = swerve;
        AutoBuilder.configureHolonomic(() -> swerveDrive.getPose(), swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds, (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)), pathFollowerConfig, () -> false, swerveDrive);

        PPHolonomicDriveController.setRotationTargetOverride(this::autoAim);

        NamedCommands.registerCommand("deployIntake", new SequentialCommandGroup(
                _intakeStepUPArm.withTimeout(0.1),
                intake.deployIntake(),
                shooter.runShooterAtPercent(-.3),
                indexer.setIndexerSpeed(-.4),
                intake.setIntakeSpeed(0.9),
                _intakeArm.withTimeout(0.2)
        ));

        NamedCommands.registerCommand("retractIntake", new SequentialCommandGroup(
                intake.setIntakeSpeed(-.9),
                new WaitCommand(0.2),
                _backUpArm.withTimeout(0.2),
                shooter.runShooterAtPercent(0),
                intake.stowIntake(),
                indexer.setIndexerSpeed(-0.1),
                new WaitCommand(0.1),
                indexer.setIndexerSpeed(-0.05),
                intake.setIntakeSpeed(0),
                _upStableArm.withTimeout(0.2)
        ));

        NamedCommands.registerCommand("halfDeployIntake", new SequentialCommandGroup(
                shooter.runShooterAtPercent(-.3),
                indexer.setIndexerSpeed(-.4),
                intake.setIntakeSpeed(0.9),
                _halfIntakeArm.withTimeout(0.6)
        ));

        NamedCommands.registerCommand("halfRetractIntake", new SequentialCommandGroup(intake.setIntakeSpeed(-.9),
                intake.setIntakeSpeed(-.9),
                new WaitCommand(0.2),
                _halfIntakeBackShot.withTimeout(0.6),
                shooter.runShooterAtPercent(0),
                indexer.setIndexerSpeed(-0.1),
                new WaitCommand(0.1),
                indexer.setIndexerSpeed(-0.05),
                intake.setIntakeSpeed(0)
        ));

//        NamedCommands.registerCommand("bottomMidShot", new SequentialCommandGroup(
//                arm.setArmSetpoint(53),
//                new WaitCommand(1),
//                shooter.runShooterAtPercent(.6),
//                new WaitCommand(1),
//                indexer.setIndexerSpeed(0.2),
//                new WaitCommand(0.2),
//                shooter.runShooterAtPercent(0)
//        ));

//        NamedCommands.registerCommand("shuttleShot", new SequentialCommandGroup(arm.setArmSetpoint(23), new WaitCommand(.3), robotContainer.runAutoShooting(), new WaitCommand(.1), arm.rotateStable()));
//        NamedCommands.registerCommand("halfBackShot", new SequentialCommandGroup(arm.setArmSetpoint(147 /*141*/), new WaitCommand(0.25), robotContainer.runAutoShooting()));
        NamedCommands.registerCommand("halfBackShot", new SequentialCommandGroup(
                shooter.runShooterAtRpm(4000).alongWith(indexer.setIndexerSpeed(-0.1))
                , new WaitCommand(0.7), indexer.setIndexerSpeed(0.4),
                shooter.runShooterAtPercent(0), indexer.setIndexerSpeed(0)));
        NamedCommands.registerCommand("backShot", new SequentialCommandGroup(
                _backShot.withTimeout(0.5).alongWith(shooter.runShooterAtRpm(4000))
                        .alongWith(indexer.setIndexerSpeed(-0.1)),
                new WaitCommand(0.6), indexer.setIndexerSpeed(0.4),
                shooter.runShooterAtPercent(0), indexer.setIndexerSpeed(0), _armStable.withTimeout(0.2)));
        // NamedCommands.registerCommand("backShotIntake", new SequentialCommandGroup(
        //         _backShot.withTimeout(0.5).alongWith(shooter.runShooterAtRpm(4000))
        //                 .alongWith(indexer.setIndexerSpeed(-0.1)).andThen(intake.deployIntake()),
        //         new WaitCommand(0.6), indexer.setIndexerSpeed(0.4),
        //         shooter.runShooterAtPercent(0), indexer.setIndexerSpeed(0), _armStable.withTimeout(0.2)));

        NamedCommands.registerCommand("twoPieceExtendedShotRed", new SequentialCommandGroup(
                _twoPieceExtendedShotRed.withTimeout(0.8).alongWith(shooter.runShooterAtRpm(5500))
                        .alongWith(indexer.setIndexerSpeed(-0.1)),
                new WaitCommand(0.5), indexer.setIndexerSpeed(0.4),
                shooter.runShooterAtPercent(0), indexer.setIndexerSpeed(0), _twoPieceArmStableRed.withTimeout(0.2)));
        NamedCommands.registerCommand("twoPieceExtendedShotBlue", new SequentialCommandGroup(
                _twoPieceExtendedShotBlue.withTimeout(0.8).alongWith(shooter.runShooterAtRpm(5500))
                        .alongWith(indexer.setIndexerSpeed(-0.1)),
                new WaitCommand(0.75), indexer.setIndexerSpeed(0.4),
                shooter.runShooterAtPercent(0), indexer.setIndexerSpeed(0), _twoPieceArmStableBlue.withTimeout(0.2)));
//        NamedCommands.registerCommand("topShot", new SequentialCommandGroup(arm.setArmSetpoint(61), new WaitCommand(0.2), robotContainer.runAutoShooting(), new WaitCommand(.2), arm.rotateStable()));
//        NamedCommands.registerCommand("midShot", new SequentialCommandGroup(arm.setArmSetpoint(61.5), new WaitCommand(0.5), robotContainer.runAutoShooting(), new WaitCommand(0.2), arm.rotateStable()));
//        NamedCommands.registerCommand("bottomShot", new SequentialCommandGroup(arm.setArmSetpoint(57.5), new WaitCommand(0.5), robotContainer.runAutoShooting(), new WaitCommand(0.2), arm.rotateStable()));
//        NamedCommands.registerCommand("bottomFarShot", new SequentialCommandGroup(arm.setArmSetpoint(45.5), new WaitCommand(0.5), robotContainer.autoFarShot(), new WaitCommand(0.2), arm.rotateStable()));

//        NamedCommands.registerCommand("autoAim", new SequentialCommandGroup(arm.isAutoAiming(true), new WaitCommand(0.3), shooter.runShooterAtRpm(5300), new WaitCommand(2), indexer.setIndexerSpeed(0.2)));
//        NamedCommands.registerCommand("stopAutoAim", new SequentialCommandGroup(arm.isAutoAiming(false), shooter.runShooterAtPercent(0), indexer.setIndexerSpeed(0)));
//        NamedCommands.registerCommand("autoAimOff", new SequentialCommandGroup(runOnce(() -> enableAutoAim = false), arm.isAutoAiming(false)));

//        NamedCommands.registerCommand("autoShoot", new SequentialCommandGroup(arm.isAutoAiming(true), new InstantCommand(() -> shooter.autoTargeting = true), shooter.runShooterPredeterminedRPM(),
//                new WaitCommand(0.7), indexer.setIndexerSpeed(0.2),
//                new WaitCommand(0.2), indexer.setIndexerSpeed(0), arm.isAutoAiming(false)));

       Shuffleboard.getTab("Autons").add("Side", side);
       side.addOption("Red Side", true);
       side.addOption("Blue Side", false);

       Shuffleboard.getTab("Autons").add("Auton Style Red", autonChooserRed).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
       autonChooserRed.addOption("doNothing", doNothing());
//        autonChooserRed.addOption("move do nothing", new SequentialCommandGroup(arm.setArmSetpoint(141), new WaitCommand(0.5), robotContainer.runAutoShooting(), new WaitCommand(0.5), arm.rotateSafe(), shooter.runShooterAtPercent(0), indexer.setIndexerSpeed(0), arm.isAiming(false)));
       autonChooserRed.addOption("testBackShot", testBackShot());

       autonChooserRed.addOption("onePieceTaxiTopRed", onePieceTaxiTopRed());
       autonChooserRed.addOption("onePieceTaxiMiddleRed", onePieceTaxiMiddleRed());
       autonChooserRed.addOption("onePieceTaxiBottomRed", onePieceTaxiBottomRed());
       autonChooserRed.addOption("oneAndHalfPieceTaxiBottomRed", oneAndHalfPieceTaxiBottomRed());

       autonChooserRed.addOption("twoPieceTopRed", twoPieceTopRed());
       autonChooserRed.addOption("twoPieceMiddleRed", twoPieceMiddleRed());
       autonChooserRed.addOption("twoPieceBottomRed", twoPieceBottomRed());
       autonChooserRed.addOption("twoPieceBottomFarRed", twoAndHalfPieceExtendedRed());
        autonChooserBlue.addOption("twoPieceExtendedRed", twoPieceExtendedRed());


       autonChooserRed.addOption("threePieceMtBRed", threePieceMtBRed());
       autonChooserRed.addOption("threePieceMtTRed", threePieceMtTRed());

       //        autonChooserRed.addOption("threePieceTtMRed", threePieceTtMRed());
       //        autonChooserRed.addOption("threePieceTtMAutoAimRed", threePieceTtMAutoAimRed());
       //        autonChooserRed.addOption("threePieceBtMAutoAimRed", threePieceBtMAutoAimRed());
       //        autonChooserRed.addOption("threePieceBottomFarAutoAimRed", threePieceBottomFarAutoAimRed());

       autonChooserRed.addOption("fourPieceMiddleTtBRed", fourPieceMiddleTtBRed());
       autonChooserRed.addOption("fourPieceTtBAutoAimRed", fourPieceTtBAutoAimRed());
       autonChooserRed.addOption("fourPieceTopFarAutoAimRed", fourPieceTopFarAutoAimRed());

       Shuffleboard.getTab("Autons").add("Auton Style Blue", autonChooserBlue).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
       autonChooserBlue.addOption("doNothing", doNothing());
//        autonChooserBlue.addOption("move do nothing", new SequentialCommandGroup(arm.setArmSetpoint(141), new WaitCommand(0.5), robotContainer.runAutoShooting(), new WaitCommand(0.5), arm.rotateSafe(), shooter.runShooterAtPercent(0), indexer.setIndexerSpeed(0), arm.isAiming(false)));

       autonChooserBlue.addOption("onePieceTaxiTopBlue", onePieceTaxiTopBlue());
       autonChooserBlue.addOption("onePieceTaxiMiddleBlue", onePieceTaxiMiddleBlue());
       autonChooserBlue.addOption("onePieceTaxiBottomBlue", onePieceTaxiBottomBlue());
       autonChooserBlue.addOption("oneAndHalfPieceTaxiBottomBlue", oneAndHalfPieceTaxiBottomBlue());
       autonChooserBlue.addOption("twoPieceBottomFarBlue", twoPieceBottomFarBlue());

       autonChooserBlue.addOption("twoPieceTopBlue", twoPieceTopBlue());
       autonChooserBlue.addOption("twoPieceMiddleBlue", twoPieceMiddleBlue());
       autonChooserBlue.addOption("twoPieceBottomBlue", twoPieceBottomBlue());
       autonChooserBlue.addOption("twoAndHalfExtendedBlue", twoAndHalfExtendedBlue());
       autonChooserBlue.addOption("twoPieceExtendedBlue", twoPieceExtendedBlue());
       autonChooserBlue.addOption("twoPieceMiddleExtendedBlue", twoAndHalfPieceMiddleExtendedBlue());

       autonChooserBlue.addOption("threePieceBtMBlue", threePieceBtMBlue());
       autonChooserBlue.addOption("threePieceTtMBlue", threePieceTtMBlue());
       autonChooserBlue.addOption("threePieceMtTBlue", threePieceMtTBlue());
       autonChooserBlue.addOption("threePieceMtBBlue", threePieceMtBBlue());
    //    autonChooserBlue.addOption("threePieceExtendedBlue", threePieceExtendedBlue());

       autonChooserBlue.addOption("fourPieceTtBBlue", fourPieceMiddleTtBBlue());
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
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return Optional.of(aprilTags.autonAimRed());
            } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                return Optional.of(aprilTags.autonAimBlue());
            } else {
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

    public Command testCommand() {
        return AutoBuilder.buildAuto("testCommand");
    }

    //One piece Autons
    public Command testBackShot() {
        return AutoBuilder.buildAuto("backShot");
    }

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

    public Command twoAndHalfExtendedBlue() {
        return AutoBuilder.buildAuto("2.5 Piece Extended Blue");
    }

    public Command twoPieceBottomFarBlue() {
        return AutoBuilder.buildAuto("2 Piece Far Bottom Blue");
    }

    public Command twoPieceExtendedBlue() {
        return AutoBuilder.buildAuto("2 Piece Extended Blue");
    }

    public Command twoPieceExtendedRed() {
        return AutoBuilder.buildAuto("2 Piece Extended Red");
    }

    public Command twoAndHalfPieceExtendedRed() {
        return AutoBuilder.buildAuto("2.5 Piece Extended Red");
    }

    public Command twoAndHalfPieceMiddleExtendedBlue() {
        return AutoBuilder.buildAuto("2.5 Piece Middle Extended Blue");
    }

    //Three Piece Regular Autons
    public Command threePieceTtMRed() {
        return AutoBuilder.buildAuto("3 Piece Top to Middle Red");
    }

    public Command threePieceMtTRed() {
        return new PathPlannerAuto("3 Piece Middle to Top Red");
    }

    public Command threePieceMtBRed() {
        return new PathPlannerAuto("3 Piece Middle to Bottom Red");
    }

    public Command threePieceMtTBlue() {
        return new PathPlannerAuto("3 Piece Middle to Top Blue");
    }

    public Command threePieceMtBBlue() {
        return new PathPlannerAuto("3 Piece Middle to Bottom Blue");
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

    public Command threePieceExtendedBlue(){
        return AutoBuilder.buildAuto("3 Piece Extended Blue");
    }

    //Four Piece Autons
    public Command fourPieceTopFarAutoAimRed() {
        return AutoBuilder.buildAuto("4 Piece Top Far Red AUTOAIM");
    }

    public Command fourPieceMiddleTtBRed() {
        return AutoBuilder.buildAuto("4 Piece Middle TtB Red");
    }

    public Command fourPieceMiddleTtBBlue() {
        return AutoBuilder.buildAuto("4 Piece Top to Bottom Blue");
    }

    public Command fourPieceTtBAutoAimRed() {
        return AutoBuilder.buildAuto("4 Piece Top to Bottom Red AUTOAIM");
    }


    public Command test() {
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