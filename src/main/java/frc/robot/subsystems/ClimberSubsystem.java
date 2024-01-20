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
    climberJointMotor.set(climberJointPIDController.calculate(climberJointMotor.getPosition().getValue()));
    climberClawMotor.set(climberClawPIDController.calculate(climberClawMotor.getPosition().getValue()));
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
    return this.runOnce(() -> climberClawPIDController.setSetpoint(target));
  }

  public Command climbClimber() {
    return this.runOnce(() -> climb());
  }

  public Command storeClimber() {
    return this.runOnce(() -> store());
  }

  public void climb() {
    climberJointPIDController.setSetpoint(MainConstants.Setpoints.CLIMBER_JOINT_CLIMB_SETPOINT);
    climberClawPIDController.setSetpoint(MainConstants.Setpoints.CLIMBER_CLAW_CLIMB_SETPOINT);
  }

  public void store() {
    climberJointPIDController.setSetpoint(MainConstants.Setpoints.CLIMBER_JOINT_STORE_SETPOINT);
    climberClawPIDController.setSetpoint(MainConstants.Setpoints.CLIMBER_JOINT_STORE_SETPOINT);
  }
}