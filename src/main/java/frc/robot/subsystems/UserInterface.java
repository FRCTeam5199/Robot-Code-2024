package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.*;;

public class UserInterface extends SubsystemBase {
    NetworkTableInstance dashboard = NetworkTableInstance.getDefault();


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
    }
    

    public String getConfig(){
        return config.getSelected();
    }

}
    





    

