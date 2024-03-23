package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parsers.PivotParser;
import frc.robot.subsystems.minor.TagalongPivot;
import frc.robot.utils.PivotAugment;
import frc.robot.utils.TagalongMinorSystemBase;
import frc.robot.utils.TagalongSubsystemBase;

public class ArmSubsystemVer2 extends TagalongSubsystemBase  implements PivotAugment {
    public final PivotParser pivotParser;
    private final TagalongPivot pivot;
    public static ArmSubsystemVer2 armSubsystem;
    public TalonFX follower;

    public  ArmSubsystemVer2() {
        this(new PivotParser(Filesystem.getDeployDirectory(), "compbotShooterPivotConf.json"));
    }

    public ArmSubsystemVer2(PivotParser parser) {
      super(parser);
        pivotParser = parser;

        pivot = new TagalongPivot(pivotParser);
        configShuffleboard();
    }

    @Override
    public TagalongPivot getPivot() {
        return pivot;
    }

    @Override
    public TagalongPivot getPivot(int i) {
        return pivot;
    }
    
    /**
     * Gets the instnace of the Arm Subsystem.
     */
    public static ArmSubsystemVer2 getInstance() {
        if (armSubsystem == null) {
            armSubsystem = new ArmSubsystemVer2();
        }

        return armSubsystem;
    }

  public void onEnable() {
    pivot.onEnable();

    // for testing
    // _elevator.getElevatorMotor().setControl(
    //     new VelocityVoltage(0.000001).withFeedForward(_elevator._elevatorFF.ks)
    // );
    // System.out.println("ks " + _elevator._elevatorFF.ks);
  }

  public void onDisable() {
    pivot.onDisable();
  }

  public void periodic() {
    pivot.periodic();
    updateShuffleboard();
  }

  public void disabledPeriodic() {
    pivot.disabledPeriodic();
  }

  public void simulationInit() {
    pivot.simulationInit();
  }

  public void simulationPeriodic() {
    pivot.simulationPeriodic();
  }

  public void updateShuffleboard() {
    pivot.updateShuffleboard();
  }

  public void configShuffleboard() {
    pivot.configShuffleboard();
  }

  public boolean checkInitStatus() {
    return pivot.checkInitStatus();
  }
}
