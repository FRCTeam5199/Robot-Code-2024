package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Main;
import frc.robot.constants.MainConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase {
  public CANSparkMax ArmMotor; 
  public double rotateSetpoint = 0;
  private boolean isFront = false;
  private boolean isStable = false;
  private boolean isHigh = false;
	PIDController rotatePIDController;
	
  public ArmSubsystem() {

  }

  public void init(){
	ArmMotor = new CANSparkMax(MainConstants.IDs.Motors.ARM_LEADER_MOTOR_ID, MotorType.kBrushless);//idk if brushed or brushless

	ArmMotor.getEncoder().setPosition(0);

	rotatePIDController = new PIDController(MainConstants.PIDConstants.ARM_FOLLOWER_PID.P, MainConstants.PIDConstants.ARM_FOLLOWER_PID.I,
				MainConstants.PIDConstants.ARM_FOLLOWER_PID.D);
  }

  @Override
  public void periodic() {
    ArmMotor.set(rotatePIDController.calculate(ArmMotor.getEncoder().getPosition()*1D, rotateSetpoint));
  }

	public void rotateHumanPlayer() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_ROTATE_SETPOINT_HUMANPLAYER;
		this.isFront = true;
		this.isHigh = false;
	}

	public void rotateStable() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_ROTATE_SETPOINT_STABLE;
		this.isFront = true;
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