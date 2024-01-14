package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;

public class IntakeSubsystem extends SubsystemBase {
  public CANSparkMax intakeMotor;

  public IntakeSubsystem() {
    init();
  } 

  public void init() {
    motorInit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void motorInit() {
    intakeMotor = new CANSparkMax(MainConstants.IDs.Motors.INTAKE_MOTOR_ID, MotorType.kBrushed);

    intakeMotor.getEncoder().setPosition(0);
  }

  public Command setIntakeSpeed(double percent) {
    return this.runOnce(() -> intakeMotor.set(percent));
  }
}