package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Main;
import frc.robot.abstractMotorInterfaces.TalonMotorController;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase {
	public VortexMotorController ArmMotor;
	
	 public TalonMotorController ArmLeader;
	  public TalonMotorController ArmFollower;

	public double rotateSetpoint = 0;
	private boolean isFront = true;
	private boolean isStable = false;
	private boolean isHigh = false;
	PIDController rotatePIDController;

	public ArmSubsystem() {
		init();
	}

	public void init(){
		ArmMotor = new VortexMotorController(MainConstants.IDs.Motors.ARM_LEADER_MOTOR_ID);//idk if brushed or brushless
		//ArmFollower = new TalonMotorController(MainConstants.IDs.Motors.ARM_FOLLOWER_MOTOR_ID);


		ArmMotor.getEncoder().setPosition(0);

		rotatePIDController = new PIDController(MainConstants.PIDConstants.ARM_PID.P, MainConstants.PIDConstants.ARM_PID.I,
				MainConstants.PIDConstants.ARM_PID.D);
		ArmMotor.setInvert(true);
		ArmMotor.setBrake(false);
	}

	@Override
	public void periodic() {
		if (ArmMotor.getEncoder().getPosition() < 3) {
			while (ArmMotor.getEncoder().getPosition() < 3) {
					ArmMotor.set(0.5);
			}
			
			ArmMotor.set(0);
			this.rotateSetpoint = ArmMotor.getEncoder().getPosition();
		} else if (ArmMotor.getEncoder().getPosition() > 100) {
			while (ArmMotor.getEncoder().getPosition() > 100) {
				ArmMotor.set(-0.5);
			}

			ArmMotor.set(0);
			this.rotateSetpoint = ArmMotor.getEncoder().getPosition();
		} else {
				ArmMotor.set(rotatePIDController.calculate(ArmMotor.getEncoder().getPosition()*1D, rotateSetpoint));
		}
	}

	public void rotateHumanPlayer() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_ROTATE_SETPOINT_HUMANPLAYER;
		this.isFront = true;
		this.isHigh = false;
	}

	public Command moveAtPercent(double percent) {
		
		return this.run(() -> ArmMotor.set(this.rotateSetpoint));
	}
	
	public Command changeArmSetpoint(double rotations) {
    return this.runOnce(() -> this.rotateSetpoint += rotations);
  }
  
	public Command setArmSetpoint(double setpoint) {
    return this.runOnce(() -> rotatePIDController.setSetpoint(setpoint));
  }

	public void rotateStable() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_ROTATE_SETPOINT_STABLE;
		this.isFront = true;
		this.isHigh = false;
	}

	public void rotateAmp() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_ROTATE_SETPOINT_AMP;
		this.isFront = false;
		this.isHigh = false;
	}

	public void rotateHigh() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_ROTATE_SETPOINT_HIGH;
		this.isFront = false;
		this.isHigh = true;
	}

	public void rotateMid() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_ROTATE_SETPOINT_MID;
		this.isFront = false;
		this.isHigh = false;
	}

	public void rotateLow() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_ROTATE_SETPOINT_LOW;
		this.isFront = false;
		this.isHigh = false;
	}
	public boolean isFront() {
		return this.isFront;
	}
}