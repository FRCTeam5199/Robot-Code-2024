package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.MainConstants;

public class ClimberSubsystem extends SubsystemBase {
  private static ClimberSubsystem climberSubsystem;
  
  private static boolean subsystemStatus = false;

  public CANSparkMax climberMotor1;
  public CANSparkMax climberMotor2;

  public PIDController climberPIDController;

  private boolean climbModeEnabled = false;
  
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
    climberMotor1 = new CANSparkMax(MainConstants.IDs.Motors.CLIMBER_MOTOR_1_ID, MotorType.kBrushless);

    climberMotor1.getEncoder().setPosition(0);
    climberMotor1.setInverted(false);
    climberMotor1.setIdleMode(IdleMode.kBrake);
    

    climberMotor2 = new CANSparkMax(MainConstants.IDs.Motors.CLIMBER_MOTOR_2_ID, MotorType.kBrushless);

    climberMotor2.getEncoder().setPosition(0);
    climberMotor2.setInverted(true);
    climberMotor2.setIdleMode(IdleMode.kBrake);
  }

  private void PIDInit() {
    climberPIDController = new PIDController(MainConstants.PIDConstants.CLIMBER_PID.P, MainConstants.PIDConstants.CLIMBER_PID.I, MainConstants.PIDConstants.CLIMBER_PID.D);
  }

  public boolean checkMotors() {
		if (climberMotor1 != null && climberMotor2 != null) {
			return true;
		} else {
			return false;
		}
	}

	public boolean checkPID() {
		if (climberPIDController != null) {
			return true;
		} else {
			return false;
		}
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (checkMotors() && checkPID()) { subsystemStatus = true; } else { subsystemStatus = false; }
    
		if (subsystemStatus) {
      subsystemPeriodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void subsystemPeriodic() {
    // climberMotor1.set(climberPIDController.calculate(climberMotor1.getEncoder().getPosition()));
    // climberMotor2.set(climberPIDController.calculate(climberMotor2.getEncoder().getPosition()));
  }

  /*
   * Sets the climb mode of the Arm. False is TeleOp mode.
   */
  public Command setClimbMode(boolean mode){
    return this.runOnce(()-> new Command() {{
      climbModeEnabled = mode;
      // setClimberTarget(5); //Tune
    }});
  }

  /**
   * @param percent sets climber speed 
   * @return command to set climber speed
   */
  public Command setClimberSpeed(double percent) {
    return this.runOnce(()-> new Command(){{
      if (climbModeEnabled) {
        climberMotor1.set(percent);
        climberMotor2.set(percent);
      }
    }});
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
  public Command setClimberTarget(double target) {
    return this.runOnce(() -> climberPIDController.setSetpoint(target));
  }
}