package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;

public class ClimberSubsystem extends SubsystemBase {
  public CANSparkMax climberJointMotor;
  public CANSparkMax climberWristMotor;
  public CANSparkMax climberClawMotor;

  public PIDController climberJointPIDController;
  public PIDController climberWristPIDController;
  public PIDController climberClawPIDController;

  private boolean climbed = false;
  private boolean climbing = false;

  public ClimberSubsystem() {
    init();
  }
  
  public void init() {
    motorInit();
    PIDInit();
  }

  public void motorInit() {
    climberJointMotor = new CANSparkMax(MainConstants.IDs.Motors.CLIMBER_JOINT_MOTOR_ID, MotorType.kBrushed);
    climberWristMotor = new CANSparkMax(MainConstants.IDs.Motors.CLIMBER_WRIST_MOTOR_ID, MotorType.kBrushed);
    climberClawMotor = new CANSparkMax(MainConstants.IDs.Motors.CLIMBER_CLAW_MOTOR_ID, MotorType.kBrushed);

    climberJointMotor.getEncoder().setPosition(0);
    climberWristMotor.getEncoder().setPosition(0);
    climberClawMotor.getEncoder().setPosition(0);
  }
  private void PIDInit() {
    climberJointPIDController = new PIDController(MainConstants.PIDConstants.CLIMBER_JOINT_PID.P, MainConstants.PIDConstants.CLIMBER_JOINT_PID.I, MainConstants.PIDConstants.CLIMBER_JOINT_PID.D);
    climberWristPIDController = new PIDController(MainConstants.PIDConstants.CLIMBER_WRIST_PID.P, MainConstants.PIDConstants.CLIMBER_WRIST_PID.I, MainConstants.PIDConstants.CLIMBER_WRIST_PID.D);
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
  
  public Command setClimberWristSpeed(double percent) {
    return this.runOnce(() -> climberWristMotor.set(percent));
  }

  public Command setClimberClawSpeed(double percent) {
    return this.runOnce(() -> climberClawMotor.set(percent));
  }

  public Command setClimberJointTarget(double target) {
    return this.runOnce(() -> climberJointPIDController.setSetpoint(target));
  }

  public Command setClimberWristTarget(double target) {
    return this.runOnce(() -> climberJointPIDController.setSetpoint(target));
  }

  public Command setClimberClawTarget(double target) {
    return this.runOnce(() -> climberJointPIDController.setSetpoint(target));
  }

  public Command runClimber() {
    return this.runOnce(() -> startClimb());
  }

  public Command fullClimber() {
    return this.runOnce(() -> fullClimb());
  }

  public Command storeClimber() {
    return this.runOnce(() -> storeClimb());
  }

  public void startClimb() {
    climberJointPIDController.setSetpoint(0);
    climberWristPIDController.setSetpoint(0);
    climberClawPIDController.setSetpoint(0);

    this.climbed = false;
    this.climbing = true;
  }

  public void fullClimb() {
    climberJointPIDController.setSetpoint(0);
    climberWristPIDController.setSetpoint(0);
    climberClawPIDController.setSetpoint(0);

    this.climbed = true;
    this.climbing = false;
  }

  public void storeClimb() {
    climberJointPIDController.setSetpoint(0);
    climberWristPIDController.setSetpoint(0);
    climberClawPIDController.setSetpoint(0);

    this.climbed = false;
    this.climbing = false;
  }

  public boolean isClimbed() {
    return this.climbed;
  }

  public boolean isClimbing() {
    return this.climbing;
  }
}