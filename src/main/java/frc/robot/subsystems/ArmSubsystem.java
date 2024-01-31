package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstractMotorInterfaces.TalonMotorController;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;


public class ArmSubsystem extends SubsystemBase {
	public VortexMotorController ArmMotor;
	
	 public TalonMotorController ArmLeader;
	public TalonMotorController ArmFollower;
	public RelativeEncoder relativeEncoder;

	public double rotateSetpoint = 0;
	PIDController rotatePIDController;

	public ArmSubsystem() {}

	/**
	 * init for arm and pid controller
	 */
	public void init(){
	
		motorInit();
		PIDInit();


		Shuffleboard.getTab("Status").add("Arm Subsystem Status", true).getEntry();
	}

	public void motorInit() {
		ArmMotor = new VortexMotorController(MainConstants.IDs.Motors.ARM_MOTOR_ID);

		
		ArmMotor.setInvert(true);
		ArmMotor.setBrake(true);

		ArmMotor.getEncoder().setPosition(0);

		

	}

	public void PIDInit() {
				rotatePIDController = new PIDController(MainConstants.PIDConstants.ARM_PID.P, MainConstants.PIDConstants.ARM_PID.I,
				MainConstants.PIDConstants.ARM_PID.D);
	}

	@Override
	public void periodic() {
		
		if (ArmMotor.getEncoder().getPosition() < 1.5) {
			while (ArmMotor.getEncoder().getPosition() < 1.5) {
					ArmMotor.set(0.3);
			}
			
			ArmMotor.set(0);
			this.rotateSetpoint = ArmMotor.getEncoder().getPosition();

		} else if (ArmMotor.getEncoder().getPosition() > 61) {
			while (ArmMotor.getEncoder().getPosition() > 61) {
				ArmMotor.set(-0.3);
			}

			ArmMotor.set(0);
			this.rotateSetpoint = ArmMotor.getEncoder().getPosition();
		} else {

				// ArmMotor.set(rotatePIDController.calculate(ArmMotor.getEncoder().getPosition(), rotateSetpoint));
		}
	}

	/**
	 * 
	 * @param percent move armMotor at a percent(-1 to 1)
	 * @return command to spin motor at percent
	 */
	public Command moveAtPercent(double percent) {
		return this.run(() -> ArmMotor.set(percent));
	}
	
	/**
	 * 
	 * @param rotations adds this to setPoint variable(setpoint)
	 * @return command to move to setPoint
	 */
	public Command changeArmSetpoint(double rotations) {
    System.out.println(rotateSetpoint + rotations);
    return this.runOnce(() -> this.rotateSetpoint += rotations);
  }
  
    /**
	 * 
	 * @param setpoint set armMotor to setPoint
	 * @return command to move armMotor to setPoint
	 */
	public Command setArmSetpoint(double setpoint) {
    return this.runOnce(() -> rotatePIDController.setSetpoint(setpoint));
  }
  	/**
	 * set Setpoint variable to Stable
	 */
	public void rotateStable() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_STABLE_SETPOINT;
	}

	/**
	 * set Setpoint variable to back
	 */
	public void rotateBack() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_SPEAKER_BACK_SETPOINT;
	}
	
	/**
	 * set Setpoint variable to front
	 */
	public void rotateFront() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_SPEAKER_FRONT_SETPOINT;
	}

	/**
	 * set Setpoint variable to slightly above retracted intake
	 */
	public void rotateIntake() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_INTAKE_SETPOINT;
	}

	/**
	 * set Setpoint variable to slightly above retracted intake
	 */
	public void rotateClimb() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_CLIMBER_SETPOINT;
	}
}