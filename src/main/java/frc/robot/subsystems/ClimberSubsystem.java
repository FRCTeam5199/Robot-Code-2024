package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.MainConstants;

public class ClimberSubsystem implements Subsystem {
  public TalonFX climberJointMotor;
  public TalonFX climberClawMotor;

  public PIDController climberJointPIDController;
  public PIDController climberClawPIDController;

  public ClimberSubsystem() {
    init();
  }
  
  public void init() {
    motorInit();
    PIDInit();
  }

  public void motorInit() {
    climberJointMotor = new TalonFX(MainConstants.IDs.Motors.CLIMBER_JOINT_MOTOR_ID);
    climberClawMotor = new TalonFX(MainConstants.IDs.Motors.CLIMBER_CLAW_MOTOR_ID);

    climberJointMotor.setPosition(0);
    climberClawMotor.setPosition(0);
  }

  private void PIDInit() {
    climberJointPIDController = new PIDController(MainConstants.PIDConstants.CLIMBER_JOINT_PID.P, MainConstants.PIDConstants.CLIMBER_JOINT_PID.I, MainConstants.PIDConstants.CLIMBER_JOINT_PID.D);
    climberClawPIDController = new PIDController(MainConstants.PIDConstants.CLIMBER_CLAW_PID.P, MainConstants.PIDConstants.CLIMBER_CLAW_PID.I, MainConstants.PIDConstants.CLIMBER_CLAW_PID.D);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Command setClimberJointSpeed(double percent) {
    return this.runOnce(() -> climberJointMotor.set(percent));
  }

  public Command setClimberClawSpeed(double percent) {
    return this.runOnce(() -> climberClawMotor.set(percent));
  }

  public Command setClimberJointTarget(double target) {
    return this.runOnce(() -> climberJointPIDController.setSetpoint(target));
  }

  public Command setClimberClawTarget(double target) {
    return this.runOnce(() -> climberJointPIDController.setSetpoint(target));
  }

  public Command climbClimber() {
    return this.runOnce(() -> startClimb());
  }

  public Command storeClimber() {
    return this.runOnce(() -> storeClimb());
  }

  public void startClimb() {
    climberJointPIDController.setSetpoint(0);
    climberClawPIDController.setSetpoint(0);
  }

  public void fullClimb() {
    climberJointPIDController.setSetpoint(0);
    climberClawPIDController.setSetpoint(0);
  }

  public void storeClimb() {
    climberJointPIDController.setSetpoint(0);
    climberClawPIDController.setSetpoint(0);
  }
}