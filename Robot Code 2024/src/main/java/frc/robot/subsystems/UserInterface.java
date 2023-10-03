package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.*;;

public class UserInterface extends SubsystemBase {
    NetworkTableInstance dashboard = NetworkTableInstance.getDefault();

    private final ShuffleboardTab Configs = Shuffleboard.getTab("Configs");
    private final ShuffleboardTab Drive = Shuffleboard.getTab("Drive");
    private final ShuffleboardTab Auton = Shuffleboard.getTab("Auton");
    private final ShuffleboardTab Subsystems = Shuffleboard.getTab("Subsystems");
    private final ShuffleboardTab Visuals = Shuffleboard.getTab("Visuals");
    SendableChooser<String> config = new SendableChooser<>();

    
    public UserInterface(){
        config.addOption("Main Config", "Main");

        Configs.add(config);
    }

    public String getConfig(){
        return config.getSelected();
    }


    





    
}
