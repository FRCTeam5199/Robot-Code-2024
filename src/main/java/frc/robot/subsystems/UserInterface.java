package frc.robot.subsystems;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class UserInterface extends SubsystemBase {
    
    SendableChooser<String> config = new SendableChooser<>();
    
    ShuffleboardTab ShuffleboardTestTab = Shuffleboard.getTab("Test");
    static ShuffleboardTab ShuffleboardGameTab = Shuffleboard.getTab("Game");
    // Shuffleboard Test Tab Components
    private static GenericEntry shuffleboardShooterSubsystemStatusComponent, shuffleboardArmSubsystemStatusComponent, shuffleboardIntakeSubsystemStatusComponent, shuffleboardClimberSubsystemStatusComponent, shuffleboardShooterMotor1SpeedComponent, shuffleboardShooterMotor2SpeedComponent, shuffleboardShooterIndexerMotorSpeedComponent, shuffleboardArmIndexerMotorPositionComponent, shuffleboardIntakeMotorSpeedComponent, shuffleboardIntakeActuatorMotorPositionComponent;
    // Shuffleboard Game Tab Components
    private static GenericEntry shuffleboardEventNameComponent, shuffleboardMatchNumberComponent, shuffleboardMatchTypeComponent, shuffleboardAllianceColorComponent, shuffleboardStationNumberComponent, shuffleboardGameMessageComponent, shuffleboardRobotEnabledComponent, shuffleboardTeleopEnabledComponent, shuffleboardCameraComponent, shuffleboardFieldComponent;
    
    public UserInterface() {
        Shuffleboard.getTab("Drive");
        Shuffleboard.getTab("Auton");
        Shuffleboard.getTab("Visual");
        config.addOption("Main Config", "Main");

        Shuffleboard.getTab("Configs").add("Config Options", config)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

        

    }

    public void initalizeTestTab() {
        shuffleboardShooterSubsystemStatusComponent = ShuffleboardTestTab.add("Shooter Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1).withSize(2, 1).getEntry();
        
        shuffleboardArmSubsystemStatusComponent = ShuffleboardTestTab.add("Arm Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(2, 1).withSize(2, 1).getEntry();
        
        shuffleboardIntakeSubsystemStatusComponent = ShuffleboardTestTab.add("Intake Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(4, 1).withSize(2, 1).getEntry();
        
        shuffleboardClimberSubsystemStatusComponent = ShuffleboardTestTab.add("Climber Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(6, 1).withSize(2, 1).getEntry();


        shuffleboardShooterMotor1SpeedComponent = ShuffleboardTestTab.add("Shooter Motor 1 Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 1).withSize(1, 1).getEntry();
        shuffleboardShooterMotor2SpeedComponent = ShuffleboardTestTab.add("Shooter Motor 2 Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(1, 1).withSize(1, 1).getEntry();
        shuffleboardShooterIndexerMotorSpeedComponent = ShuffleboardTestTab.add("Shooter Indexer Motor Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 2).withSize(1, 1).getEntry();

        shuffleboardArmIndexerMotorPositionComponent = ShuffleboardTestTab.add("Arm Motor Position", 0)
        .withWidget(BuiltInWidgets.kEncoder)
        .withPosition(2, 1).withSize(2, 2).getEntry();

        shuffleboardIntakeMotorSpeedComponent = ShuffleboardTestTab.add("Intake Motor Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(3, 1).withSize(1, 1).getEntry();
        shuffleboardIntakeActuatorMotorPositionComponent = ShuffleboardTestTab.add("Intake Actuator Motor Position", 0)
        .withWidget(BuiltInWidgets.kEncoder)
        .withPosition(3, 2).withSize(2, 2).getEntry();
    }
    public void initalizeGameTab() {
        shuffleboardEventNameComponent = ShuffleboardGameTab.add("Event Name", "")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(2, 1)
        .withPosition(0, 0).getEntry();
        shuffleboardMatchNumberComponent = ShuffleboardGameTab.add("Match Number", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(2, 0).getEntry();
        shuffleboardMatchTypeComponent = ShuffleboardGameTab.add("Match Type", "")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(3, 0).getEntry();
        shuffleboardAllianceColorComponent = ShuffleboardGameTab.add("Alliance Color", "")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(4, 0).getEntry();
        shuffleboardStationNumberComponent = ShuffleboardGameTab.add("Station Number", "")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(5, 0).getEntry();
        shuffleboardGameMessageComponent = ShuffleboardGameTab.add("Game Message", "")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(5, 1)
        .withPosition(6, 0).getEntry();

        shuffleboardRobotEnabledComponent = ShuffleboardGameTab.add("Robot Enabled", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1)
        .withPosition(0, 1).getEntry();
        shuffleboardTeleopEnabledComponent = ShuffleboardGameTab.add("Teleop Enabled", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1)
        .withPosition(1, 1).getEntry();

        shuffleboardCameraComponent = ShuffleboardGameTab.add("Camera", "")
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(0, 2)
        .withSize(5, 5).getEntry();

        shuffleboardFieldComponent = ShuffleboardGameTab.add("Field", "")
        .withWidget(BuiltInWidgets.kField)
        .withPosition(5, 2)
        .withSize(5, 5).getEntry();
    }

    public void updateTestTab(double shooterMotor1Speed, double shooterMotor2Speed, double shooterIndexerMotorSpeed, double intakeMotorSpeed) {
        shuffleboardShooterMotor1SpeedComponent.setInteger((long)shooterMotor1Speed);
        shuffleboardShooterMotor2SpeedComponent.setInteger((long)shooterMotor2Speed);
        shuffleboardShooterIndexerMotorSpeedComponent.setInteger((long)shooterIndexerMotorSpeed);

        shuffleboardIntakeMotorSpeedComponent.setInteger((long)intakeMotorSpeed);
    }
    public void updateGameTab() {
        shuffleboardEventNameComponent.setString(DriverStation.getEventName());
        shuffleboardMatchNumberComponent.setInteger(DriverStation.getMatchNumber());
        shuffleboardMatchTypeComponent.setString(DriverStation.getMatchType().name());
        shuffleboardAllianceColorComponent.setString(DriverStation.getAlliance().toString());
        shuffleboardStationNumberComponent.setString(DriverStation.getLocation().toString());
        shuffleboardGameMessageComponent.setString(DriverStation.getGameSpecificMessage());
        
        shuffleboardRobotEnabledComponent.setBoolean(DriverStation.isEnabled());
        shuffleboardTeleopEnabledComponent.setBoolean(DriverStation.isTeleopEnabled());
        
        // shuffleboardCameraComponent.setValue("");
        // shuffleboardFieldComponent.setValue("");
    }

    /**
     * @return The selected config
     */
    public String getConfig() {
        return config.getSelected();
    }
}
