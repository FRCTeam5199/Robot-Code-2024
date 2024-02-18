package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class UserInterface extends SubsystemBase {
    SendableChooser<String> config = new SendableChooser<>();
    
    Field2d field = new Field2d();
    // SwerveDriveState swerveDriveState = new SwerveDriveState();
    // Pose2d swerveDrivePose = swerveDriveState.Pose;
    SwerveDrive swerveDrive = TunerConstants.DriveTrain;
    Pose2d swerveDrivePose = swerveDrive.robotPose;
    
    ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    ArmSubsystem armSubsystem = new ArmSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    // Autos autos = new Autos(swerveDrive, intakeSubsystem, armSubsystem, shooterSubsystem);

    public ShuffleboardTab shuffleboardTestTab;
    public ShuffleboardTab shuffleboardGameTab;// = Shuffleboard.getTab("Game");;
    // Shuffleboard Test Tab Components
    private static GenericEntry shuffleboardShooterSubsystemStatusComponent, shuffleboardArmSubsystemStatusComponent, shuffleboardIntakeSubsystemStatusComponent, shuffleboardClimberSubsystemStatusComponent, shuffleboardShooterMotor1SpeedComponent, shuffleboardShooterMotor2SpeedComponent, shuffleboardShooterIndexerMotorSpeedComponent, shuffleboardArmIndexerMotorEncoderComponent, shuffleboardIntakeMotorSpeedComponent, shuffleboardIntakeActuatorMotorEncoderComponent;
    // Shuffleboard Game Tab Components
    private static GenericEntry shuffleboardEventNameComponent, shuffleboardMatchNumberComponent, shuffleboardMatchTypeComponent, shuffleboardAllianceColorComponent, shuffleboardStationNumberComponent, shuffleboardGameMessageComponent, shuffleboardRobotEnabledComponent, shuffleboardEmergencyStoppedComponent, shuffleboardAutonomousComponent, shuffleboardTeleopComponent, shuffleboardCameraComponent;
    
    public UserInterface() {
        // Shuffleboard.getTab("Drive");
        // Shuffleboard.getTab("Auton");
        // Shuffleboard.getTab("Visual");
    }

    public void initalizeConfigTab() {
        config.addOption("Main Config", "Main");

        Shuffleboard.getTab("Configs").add("Config Options", config)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

        // Removes annoying warning that joystick port is disconnected
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void initalizeTestTab() {
        shuffleboardTestTab = Shuffleboard.getTab("Test");

        shuffleboardShooterSubsystemStatusComponent = shuffleboardTestTab.add("Shooter Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 0)
        .withSize(2, 1).getEntry();

        shuffleboardArmSubsystemStatusComponent = shuffleboardTestTab.add("Arm Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(2, 0)
        .withSize(2, 1).getEntry();

        shuffleboardIntakeSubsystemStatusComponent = shuffleboardTestTab.add("Intake Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(4, 0)
        .withSize(2, 1).getEntry();

        shuffleboardClimberSubsystemStatusComponent = shuffleboardTestTab.add("Climber Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(6, 0)
        .withSize(2, 1).getEntry();


        shuffleboardShooterMotor1SpeedComponent = shuffleboardTestTab.add("Shooter Motor 1 Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 1)
        .withSize(1, 1).getEntry();
        shuffleboardShooterMotor2SpeedComponent = shuffleboardTestTab.add("Shooter Motor 2 Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(1, 1)
        .withSize(1, 1).getEntry();
        shuffleboardShooterIndexerMotorSpeedComponent = shuffleboardTestTab.add("Shooter Indexer Motor Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 2)
        .withSize(1, 1).getEntry();

        shuffleboardArmIndexerMotorEncoderComponent = shuffleboardTestTab.add("Arm Motor Encoder", 0)
        .withWidget(BuiltInWidgets.kEncoder)
        .withPosition(2, 1)
        .withSize(2, 2).getEntry();

        shuffleboardIntakeMotorSpeedComponent = shuffleboardTestTab.add("Intake Motor Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(3, 1)
        .withSize(1, 1).getEntry();
        shuffleboardIntakeActuatorMotorEncoderComponent = shuffleboardTestTab.add("Intake Actuator Motor Encoder", 0)
        .withWidget(BuiltInWidgets.kEncoder)
        .withPosition(3, 2)
        .withSize(2, 2).getEntry();
    }

    public void initalizeGameTab() {
        shuffleboardGameTab = Shuffleboard.getTab("Game");
        
        if (DriverStation.isFMSAttached()) {
            shuffleboardEventNameComponent = shuffleboardGameTab.add("Event Name", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(2, 1).getEntry();
            shuffleboardMatchNumberComponent = shuffleboardGameTab.add("Match Number", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 0)
            .withSize(1, 1).getEntry();
            shuffleboardMatchTypeComponent = shuffleboardGameTab.add("Match Type", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(3, 0)
            .withSize(1, 1).getEntry();
            shuffleboardAllianceColorComponent = shuffleboardGameTab.add("Alliance Color", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 0)
            .withSize(1, 1).getEntry();
            shuffleboardStationNumberComponent = shuffleboardGameTab.add("Station Number", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(5, 0)
            .withSize(1, 1).getEntry();
            shuffleboardGameMessageComponent = shuffleboardGameTab.add("Game Message", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(6, 0)
            .withSize(5, 1).getEntry();
        }

        shuffleboardRobotEnabledComponent = shuffleboardGameTab.add("Robot Enabled", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1)
        .withSize(1, 1).getEntry();
        shuffleboardEmergencyStoppedComponent = shuffleboardGameTab.add("Emergency Stopped", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(1, 1)
        .withSize(1, 1).getEntry();
        shuffleboardAutonomousComponent = shuffleboardGameTab.add("Autonomous", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(2, 1)
        .withSize(1, 1).getEntry();
        shuffleboardTeleopComponent = shuffleboardGameTab.add("TeleOperated", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(3, 1)
        .withSize(1, 1).getEntry();

        shuffleboardCameraComponent = shuffleboardGameTab.add("Camera", "")
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(0, 2)
        .withSize(3, 2).getEntry();

        shuffleboardGameTab.add("Game Field", field)
        .withWidget(BuiltInWidgets.kField)
        .withPosition(6,2)
        .withSize(4, 2);

        field.setRobotPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }

    public void updateTestTab() {
        shuffleboardShooterSubsystemStatusComponent.setBoolean(shooterSubsystem.getSubsystemStatus());
        shuffleboardArmSubsystemStatusComponent.setBoolean(armSubsystem.getSubsystemStatus());
        shuffleboardIntakeSubsystemStatusComponent.setBoolean(intakeSubsystem.getSubsystemStatus());
        shuffleboardClimberSubsystemStatusComponent.setBoolean(climberSubsystem.getSubsystemStatus());

        // shuffleboardShooterMotor1SpeedComponent.setInteger((long)shooterSubsystem.getShooterMotor1Encoder().getVelocity());
        // shuffleboardShooterMotor2SpeedComponent.setInteger((long)shooterSubsystem.getShooterMotor1Encoder().getVelocity());
        // shuffleboardShooterIndexerMotorSpeedComponent.setInteger((long)shooterSubsystem.getShooterIndexerMotorEncoder().getVelocity());

        // shuffleboardIntakeMotorSpeedComponent.setInteger((long)intakeSubsystem.getIntakeMotorEncoder().getVelocity());
        // shuffleboardIntakeActuatorMotorEncoderComponent.setValue(intakeSubsystem.getIntakeActuatorMotorEncoder());
    }

    public void updateGameTab() {
        if (DriverStation.isFMSAttached()) {
            shuffleboardEventNameComponent.setString(DriverStation.getEventName());
            shuffleboardMatchNumberComponent.setInteger(DriverStation.getMatchNumber());
            shuffleboardMatchTypeComponent.setString(DriverStation.getMatchType().toString());
            shuffleboardAllianceColorComponent.setString(DriverStation.getAlliance().get().toString());
            shuffleboardStationNumberComponent.setInteger(DriverStation.getLocation().getAsInt());
            shuffleboardGameMessageComponent.setString(DriverStation.getGameSpecificMessage());
        } else {
            // shuffleboardAllianceColorComponent.setString(autos.getAutonSide().getSelected().toString());
        }

        shuffleboardRobotEnabledComponent.setBoolean(DriverStation.isEnabled());
        shuffleboardEmergencyStoppedComponent.setBoolean(DriverStation.isEStopped());
        shuffleboardAutonomousComponent.setBoolean(DriverStation.isAutonomous());
        shuffleboardTeleopComponent.setBoolean(DriverStation.isTeleop());
        
        // shuffleboardCameraComponent.setValue(CameraServer.getVideo());
        field.setRobotPose(swerveDrivePose);
    }

    /**
     * @return The selected config
     */
    public String getConfig() {
        return config.getSelected();
    }
}
