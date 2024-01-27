package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public class UserInterface extends SubsystemBase {

    SendableChooser<String> config = new SendableChooser<>();
    
    public UserInterface(){
        Shuffleboard.getTab("Drive");
        Shuffleboard.getTab("Auton");
        Shuffleboard.getTab("Visual");
        config.addOption("Main Config", "Main");

        Shuffleboard.getTab("Configs").add("Config Options", config)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

        Shuffleboard.getTab("Status").add("Shooter Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1).withSize(1, 1);
        Shuffleboard.getTab("Status").add("Arm Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1).withSize(1, 1);
        Shuffleboard.getTab("Status").add("Intake Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1).withSize(1, 1);
        Shuffleboard.getTab("Status").add("Climber Subsystem Status", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1).withSize(1, 1);
    }    

    /**
     * @return The selected config
     */
    public String getConfig() {
        return config.getSelected();
    }
}
