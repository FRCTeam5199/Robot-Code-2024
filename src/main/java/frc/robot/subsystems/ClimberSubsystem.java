package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;

public class ClimberSubsystem extends SubsystemBase {
  public CANSparkMax climberMotor;

  public ClimberSubsystem() {}

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
    climberMotor = new CANSparkMax(MainConstants.IDs.Motors.CLIMBER_MOTOR_ID, MotorType.kBrushed);

    climberMotor.getEncoder().setPosition(0);
  }

  public Command runClimber() {
    return this.runOnce(() -> climb());
  }

  public void climb() {
    climberMotor.set(0);
  }

}