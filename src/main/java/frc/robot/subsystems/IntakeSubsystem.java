package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;

public class IntakeSubsystem extends SubsystemBase {
  private DoubleSolenoid intakePiston;
//   private Compressor compressor;

  public IntakeSubsystem() {}

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
    intakePiston = new DoubleSolenoid(MainConstants.IDs.PCM_ID, MainConstants.IDs.PNEUMATICS_MODULE_TYPE, MainConstants.IDs.Pneumatics.INTAKE_IN_ID, MainConstants.IDs.Pneumatics.INTAKE_OUT_ID);
  }

  public Command lowerIntake() {
    return this.runOnce(() -> intakePiston.set(DoubleSolenoid.Value.kForward));
  }

  public Command raiseIntake() {
    return this.runOnce(() -> intakePiston.set(DoubleSolenoid.Value.kReverse));
  }
}