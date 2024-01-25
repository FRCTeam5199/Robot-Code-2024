package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public class UserInterface extends SubsystemBase {


    SendableChooser<String> config = new SendableChooser<>();

    SendableChooser<String> subsystemChooser = new SendableChooser<>();
    
    public UserInterface(){
        Shuffleboard.getTab("Drive");
        Shuffleboard.getTab("Auton");
        Shuffleboard.getTab("Visual");
        config.addOption("Main Config", "Main");
        subsystemChooser.addOption("Shooter Subsystem", "Shooter");
        subsystemChooser.addOption("Arm Subsystem", "Arm");
        subsystemChooser.addOption("Intake Subsystem", "Intake");
        subsystemChooser.addOption("Climber Subsystem", "Climber");
        
        Shuffleboard.getTab("Configs").add("Config Options", config)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);
        
        Shuffleboard.getTab("Configs").add("Config Options", config)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);
        Shuffleboard.getTab("Configs").add("Subsystem Status", subsystemChooser)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1).withSize(1, 1);
    }
    

    public String getConfig(){
        return config.getSelected();
    }
}
