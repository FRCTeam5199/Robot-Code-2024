package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

public class ClimberSubsystem extends SubsystemBase {
  private static ClimberSubsystem climberSubsystem;
  
  private static boolean subsystemStatus = false;

  public VortexMotorController climberMotor1;
  public VortexMotorController climberMotor2;

  public PIDController climberPIDController;

  public boolean PIDControlMode = false;
  
  public ClimberSubsystem() {}
  
	/** 
	 * Gets the instnace of the Arm Subsystem.
	 */
	public static ClimberSubsystem getInstance() {
		if (climberSubsystem == null) {
			climberSubsystem = new ClimberSubsystem();
		}

		return climberSubsystem;
	}

  /**
   * init for climber
   */
  public void init() {
    try { motorInit(); } catch (Exception exception) {
      System.err.println("One or more issues occured while trying to initalize motors for Climber Subsystem");
      System.err.println("Exception Message:" + exception.getMessage());
      System.err.println("Exception Cause:" + exception.getCause());
      System.err.println("Exception Stack Trace:" + exception.getStackTrace()); }

    try { PIDInit(); } catch (Exception exception) {
      System.err.println("One or more issues occured while trying to initalize PID for Climber Subsystem");
      System.err.println("Exception Message:" + exception.getMessage());
      System.err.println("Exception Cause:" + exception.getCause());
      System.err.println("Exception Stack Trace:" + exception.getStackTrace()); }
    }

  public boolean getSubsystemStatus() {
    return subsystemStatus;
  }

  /**
   * init for motor climbers 
   */
  public void motorInit() {
    climberMotor1 = new VortexMotorController(MainConstants.IDs.Motors.CLIMBER_MOTOR_1_ID);
    climberMotor2 = new VortexMotorController(MainConstants.IDs.Motors.CLIMBER_MOTOR_2_ID);

    climberMotor1.getEncoder().setPosition(0);
    climberMotor1.setInvert(false);
    climberMotor1.setBrake(true);

    climberMotor2.getEncoder().setPosition(0);
    climberMotor1.setInvert(true);
    climberMotor2.setBrake(true);
  }

  private void PIDInit() {
    // climberPIDController = new PIDController(MainConstants.PIDConstants.CLIMBER_PID.P, MainConstants.PIDConstants.CLIMBER_PID.I, MainConstants.PIDConstants.CLIMBER_PID.D);
    climberMotor1.getPIDController().setP(MainConstants.PIDConstants.CLIMBER_PID.P);
    climberMotor1.getPIDController().setP(MainConstants.PIDConstants.CLIMBER_PID.I);
    climberMotor1.getPIDController().setP(MainConstants.PIDConstants.CLIMBER_PID.D);

    climberMotor2.getPIDController().setP(MainConstants.PIDConstants.CLIMBER_PID.P);
    climberMotor2.getPIDController().setP(MainConstants.PIDConstants.CLIMBER_PID.I);
    climberMotor2.getPIDController().setP(MainConstants.PIDConstants.CLIMBER_PID.D);
  }

  public boolean checkMotors() {
		if (climberMotor1 != null && climberMotor2 != null) {
			return true;
		} else {
			return false;
		}
	}

  public RelativeEncoder getClimberMotor1Encoder() {
		if (!subsystemStatus) return null;
		return climberMotor1.getEncoder();
	}

  public RelativeEncoder getClimberMotor2Encoder() {
		if (!subsystemStatus) return null;
		return climberMotor1.getEncoder();
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (checkMotors()) { subsystemStatus = true; } else { subsystemStatus = false; }

		if (subsystemStatus) {
      subsystemPeriodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void subsystemPeriodic() {
    if (climberMotor1.getSpeed() == 0 && climberMotor2.getSpeed() == 0) {
      PIDControlMode = true;
    } else {
      PIDControlMode = false;
    }
  }

  /**
   * @param percent sets climber speed 
   * @return command to set climber speed
   */
  public Command setClimberSpeed(double percent) {
    return this.runOnce(() -> climberMotor1.set(percent)).andThen(()-> climberMotor2.set(percent));
  }

  public Command setClimberMotor1Speed(double percent) {
    return this.runOnce(() -> climberMotor1.set(percent));
  }

  public Command setClimberMotor2Speed(double percent) {
    return this.runOnce(() -> climberMotor2.set(percent));
  }

  /**
   * 
   * @param target position where motor needs to go to 
   * @return command that makes motor go to a setPositions 
   */
  public Command setClimberSetpoint(double setpoint) {
    return this.runOnce(() -> climberMotor1.setReference(setpoint)).andThen(() -> climberMotor2.setReference(setpoint));
  }

  public Command offsetClimberSetpoint(double offset) {
    return this.runOnce(() -> climberMotor1.setReference(climberMotor1.getEncoder().getPosition() + offset)).andThen(() -> climberMotor2.setReference(climberMotor1.getEncoder().getPosition() + offset));
  }

  public Command offsetClimberMotor1Setpoint(double offset) {
    return this.runOnce(() -> climberMotor1.setReference(climberMotor1.getEncoder().getPosition() + offset));
  }

  public Command offsetClimberMotor2Setpoint(double offset) {
    return this.runOnce(() -> climberMotor1.setReference(climberMotor2.getEncoder().getPosition() + offset));
  }

  public Command extendClimber() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> PIDControlMode = false),
      new InstantCommand(() -> climberMotor1.setBrake(false)),
      new InstantCommand(() -> climberMotor2.setBrake(false)),
      new WaitCommand(1),
      new InstantCommand(() -> climberMotor1.setBrake(true)),
      new InstantCommand(() -> climberMotor2.setBrake(true)),
      new InstantCommand(() -> PIDControlMode = true)
    );
  }

  public Command retractClimber() {
    PIDControlMode = true;
    return this.runOnce(() -> climberMotor1.setReference(MainConstants.Setpoints.CLIMBER_RETRACTED_SETPOINT)).andThen(() -> climberMotor2.setReference(MainConstants.Setpoints.CLIMBER_RETRACTED_SETPOINT));
  }
}