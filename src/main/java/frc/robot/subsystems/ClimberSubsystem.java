package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;

public class ClimberSubsystem extends SubsystemBase {
  private DoubleSolenoid climberPiston;

  public ClimberSubsystem() {}

  public void init() {
    pneumaticsInit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void pneumaticsInit() {
    climberPiston = new DoubleSolenoid(MainConstants.IDs.PCM_ID, MainConstants.IDs.PNEUMATICS_MODULE_TYPE, MainConstants.IDs.Pneumatics.CLIMBER_IN_ID, MainConstants.IDs.Pneumatics.CLIMBER_OUT_ID);
  }

  public Command extendClimber() {
    return this.runOnce(() -> climberPiston.set(DoubleSolenoid.Value.kForward));
  }

  public Command retractclimber() {
    return this.runOnce(() -> climberPiston.set(DoubleSolenoid.Value.kReverse));
  }
}