package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public class UserInterface extends SubsystemBase {

    SendableChooser<String> config = new SendableChooser<>();
    
    ShuffleboardTab ShuffleboardStatusTab = Shuffleboard.getTab("Status");
    static ShuffleboardTab ShuffleboardGameTab = Shuffleboard.getTab("Game");
    
    public UserInterface() {
        Shuffleboard.getTab("Drive");
        Shuffleboard.getTab("Auton");
        Shuffleboard.getTab("Visual");
        config.addOption("Main Config", "Main");

        Shuffleboard.getTab("Configs").add("Config Options", config)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);


        ShuffleboardStatusTab.add("Shooter Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1).withSize(1, 1).getEntry();
        
        ShuffleboardStatusTab.add("Arm Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(1, 1).withSize(1, 1).getEntry();
        
        ShuffleboardStatusTab.add("Intake Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(2, 1).withSize(1, 1).getEntry();
        
        ShuffleboardStatusTab.add("Climber Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(3, 1).withSize(1, 1).getEntry();

        ShuffleboardGameTab.add("Event Name", "")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(2, 1)
        .withPosition(0, 0).getEntry();
        ShuffleboardGameTab.add("Match Number", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(2, 0).getEntry();
        ShuffleboardGameTab.add("Match Type", "")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(3, 0).getEntry();
        ShuffleboardGameTab.add("Alliance Color", "")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(4, 0).getEntry();
        ShuffleboardGameTab.add("Station Number", "")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(5, 0).getEntry();
        ShuffleboardGameTab.add("Station Number", "")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(5, 1)
        .withPosition(6, 0).getEntry();

        ShuffleboardGameTab.add("Robot Enabled", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1)
        .withPosition(0, 1).getEntry();
        ShuffleboardGameTab.add("Teleop Enabled", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1)
        .withPosition(1, 1).getEntry();

        // ShuffleboardGameTab.add("Field", "").withWidget(BuiltInWidgets.kField);
    }  

    public static void updateGameTab() {
        ShuffleboardGameTab.add("Event Name", "").getEntry().setString(DriverStation.getEventName());
        ShuffleboardGameTab.add("Match Number", 0).getEntry().setInteger(DriverStation.getMatchNumber());
        ShuffleboardGameTab.add("Match Type", "").getEntry().setString(DriverStation.getMatchType().toString());
        ShuffleboardGameTab.add("Alliance Color", "").getEntry().setString(DriverStation.getAlliance().toString());
        ShuffleboardGameTab.add("Station Number", "").getEntry().setString(DriverStation.getLocation().toString());
        ShuffleboardGameTab.add("Game Message", "").getEntry().setString(DriverStation.getGameSpecificMessage());
        
        ShuffleboardGameTab.add("Robot Enabled", false).getEntry().setBoolean(DriverStation.isEnabled());
        ShuffleboardGameTab.add("Teleop Enabled", false).getEntry().setBoolean(DriverStation.isTeleopEnabled());
    }

    /**
     * @return The selected config
     */
    public String getConfig() {
        return config.getSelected();
    }
}
