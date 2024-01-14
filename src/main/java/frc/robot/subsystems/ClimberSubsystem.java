package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;

public class ClimberSubsystem extends SubsystemBase {
  public CANSparkMax climberMotor;

  public PIDController climberPIDController;

  public ClimberSubsystem() {
    init();
  }

  public void init() {
    motorInit();
    PIDInit();
  }

  private void PIDInit() {
    climberPIDController = new PIDController(MainConstants.PIDConstants.CLIMBERPID.P, MainConstants.PIDConstants.CLIMBERPID.I, MainConstants.PIDConstants.CLIMBERPID.D);
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

  public Command setClimberSpeed(double percent) {
    return this.runOnce(() -> climberMotor.set(percent));
  }

  public Command setClimberTarget(double target) {
    return this.runOnce(() -> climberPIDController.setSetpoint(target));
  }

  public Command startClimber() {
    return this.runOnce(() -> startClimb());
  }

  public Command runClimber() {
    return this.runOnce(() -> climb());
  }

  public Command storeClimber() {
    return this.runOnce(() -> storeClimb());
  }

  public void startClimb() {
    climberPIDController.setSetpoint(30);
  }

  public void climb() {
    climberPIDController.setSetpoint(20);
  }

  public void storeClimb() {
    climberPIDController.setSetpoint(0);
  }
}