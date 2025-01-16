package frc.robot.subsystems;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

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

public class UserInterface extends SubsystemBase {
    SendableChooser<String> config = new SendableChooser<>();
    
    Field2d field = new Field2d();
    SwerveDriveState swerveDriveState = new SwerveDriveState();
    Pose2d swerveDrivePose = swerveDriveState.Pose;

    ShuffleboardTab ShuffleboardTestTab = Shuffleboard.getTab("Test");
    static ShuffleboardTab ShuffleboardGameTab = Shuffleboard.getTab("Game");
    // Shuffleboard Test Tab Components
    private static GenericEntry shuffleboardShooterSubsystemStatusComponent, shuffleboardArmSubsystemStatusComponent, shuffleboardIntakeSubsystemStatusComponent, shuffleboardClimberSubsystemStatusComponent, shuffleboardShooterMotor1SpeedComponent, shuffleboardShooterMotor2SpeedComponent, shuffleboardShooterIndexerMotorSpeedComponent, shuffleboardArmIndexerMotorPositionComponent, shuffleboardIntakeMotorSpeedComponent, shuffleboardIntakeActuatorMotorPositionComponent;
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
    }

    public void initalizeTestTab() {
        shuffleboardShooterSubsystemStatusComponent = ShuffleboardTestTab.add("Shooter Subsystem Initalized", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1)
        .withSize(2, 1).getEntry();
        
        shuffleboardArmSubsystemStatusComponent = ShuffleboardTestTab.add("Arm Subsystem Initalized", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(2, 1)
        .withSize(2, 1).getEntry();
        
        shuffleboardIntakeSubsystemStatusComponent = ShuffleboardTestTab.add("Intake Subsystem Initalized", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(4, 1)
        .withSize(2, 1).getEntry();
        
        shuffleboardClimberSubsystemStatusComponent = ShuffleboardTestTab.add("Climber Subsystem Initalized", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(6, 1)
        .withSize(2, 1).getEntry();


        shuffleboardShooterMotor1SpeedComponent = ShuffleboardTestTab.add("Shooter Motor 1 Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 1)
        .withSize(1, 1).getEntry();
        shuffleboardShooterMotor2SpeedComponent = ShuffleboardTestTab.add("Shooter Motor 2 Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(1, 1)
        .withSize(1, 1).getEntry();
        shuffleboardShooterIndexerMotorSpeedComponent = ShuffleboardTestTab.add("Shooter Indexer Motor Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 2)
        .withSize(1, 1).getEntry();

        shuffleboardArmIndexerMotorPositionComponent = ShuffleboardTestTab.add("Arm Motor Position", 0)
        .withWidget(BuiltInWidgets.kEncoder)
        .withPosition(2, 1)
        .withSize(2, 2).getEntry();

        shuffleboardIntakeMotorSpeedComponent = ShuffleboardTestTab.add("Intake Motor Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(3, 1)
        .withSize(1, 1).getEntry();
        shuffleboardIntakeActuatorMotorPositionComponent = ShuffleboardTestTab.add("Intake Actuator Motor Position", 0)
        .withWidget(BuiltInWidgets.kEncoder)
        .withPosition(3, 2)
        .withSize(2, 2).getEntry();
    }

    public void initalizeGameTab() {
        if (DriverStation.isFMSAttached()) {
            shuffleboardEventNameComponent = ShuffleboardGameTab.add("Event Name", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(2, 1).getEntry();
            shuffleboardMatchNumberComponent = ShuffleboardGameTab.add("Match Number", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 0)
            .withSize(1, 1).getEntry();
            shuffleboardMatchTypeComponent = ShuffleboardGameTab.add("Match Type", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(3, 0)
            .withSize(1, 1).getEntry();
            shuffleboardAllianceColorComponent = ShuffleboardGameTab.add("Alliance Color", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 0)
            .withSize(1, 1).getEntry();
            shuffleboardStationNumberComponent = ShuffleboardGameTab.add("Station Number", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(5, 0)
            .withSize(1, 1).getEntry();
            shuffleboardGameMessageComponent = ShuffleboardGameTab.add("Game Message", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(6, 0)
            .withSize(5, 1).getEntry();

            shuffleboardRobotEnabledComponent = ShuffleboardGameTab.add("Robot Enabled", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 1)
            .withSize(1, 1).getEntry();
            shuffleboardRobotEnabledComponent = ShuffleboardGameTab.add("Emergency Stopped", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(1, 1)
            .withSize(1, 1).getEntry();
            shuffleboardAutonomousComponent = ShuffleboardGameTab.add("Autonomous", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 1)
            .withSize(1, 2).getEntry();
            shuffleboardTeleopComponent = ShuffleboardGameTab.add("TeleOperated", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 1)
            .withSize(1, 1).getEntry();
        }

        shuffleboardCameraComponent = ShuffleboardGameTab.add("Camera", "")
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(0, 2)
        .withSize(4, 3).getEntry();

        ShuffleboardGameTab.add("Game Field", field)
        .withWidget(BuiltInWidgets.kField)
        // .withPosition(6, 2)
        .withPosition(5,1)
        .withSize(5, 4);

        field.setRobotPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }

    public void updateTestTab(double shooterMotor1Speed, double shooterMotor2Speed, double shooterIndexerMotorSpeed, double intakeMotorSpeed) {
        shuffleboardShooterMotor1SpeedComponent.setInteger((long)shooterMotor1Speed);
        shuffleboardShooterMotor2SpeedComponent.setInteger((long)shooterMotor2Speed);
        shuffleboardShooterIndexerMotorSpeedComponent.setInteger((long)shooterIndexerMotorSpeed);

        shuffleboardIntakeMotorSpeedComponent.setInteger((long)intakeMotorSpeed);
    }

    public void updateGameTab() {
        if (DriverStation.isFMSAttached()) {
            shuffleboardEventNameComponent.setString(DriverStation.getEventName());
            shuffleboardMatchNumberComponent.setInteger(DriverStation.getMatchNumber());
            shuffleboardMatchTypeComponent.setString(DriverStation.getMatchType().toString());
            shuffleboardAllianceColorComponent.setString(DriverStation.getAlliance().get().toString());
            shuffleboardStationNumberComponent.setInteger(DriverStation.getLocation().getAsInt());
            shuffleboardGameMessageComponent.setString(DriverStation.getGameSpecificMessage());
        }

        // shuffleboardFieldComponent.setValue();
        // field.setRobotPose(new Pose2d(swerveDrivePose.getTranslation(), swerveDrivePose.getRotation()));
    }

    /**
     * @return The selected config
     */
    public String getConfig() {
        return config.getSelected();
    }
}
