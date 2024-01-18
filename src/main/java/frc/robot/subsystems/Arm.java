package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Main;
import frc.robot.constants.MainConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {

	PIDController rotatePIDController;


  public Arm() {
	TalonFX krakenArmFollower;
	krakenArmFollower = new TalonFX(MainConstants.krakenArmFollower)	;
	TalonFX krakenArmLeader;
	krakenArmFollower = new TalonFX(MainConstants.krakenArmLeader);
  
	public MainConstants constants = new MainConstants();

	PIDController rotatePIDController;
	TalonFX krakenArmLeader;
	TalonFX krakenArmFollower;
	
  public Arm() {
	krakenArmLeader = new TalonFX(MainConstants.IDs.Motors.ARM_LEADER_MOTOR_ID );
	krakenArmFollower = new TalonFX(MainConstants.IDs.Motors.ARM_FOLLOWER_MOTOR_ID);
	krakenArmFollower.foll

	PIDInit();
  }

  public void init(){

  }

  @Override
  public void periodic() {

	krakenArmLeader.setspeed.set(rotatePIDController.calculate(krakenArmLeader.getRotations()));

  }

  public void PIDInit() {
		rotatePIDController = new PIDController(MainConstants.PIDConstants.ARM_FOLLOWER_PID.P, MainConstants.PIDConstants.ARM_FOLLOWER_PID.I,
				MainConstants.PIDConstants.ARM_FOLLOWER_PID.D);
	}


	public Command setRotateSetpoint(int setpoint) {
		return this.runOnce(() -> rotatePIDController.setSetpoint(setpoint));
	}

	public void rotateHigh() {
		rotatePIDController.setSetpoint(-110);

	}

	public void rotateMedium() {
		rotatePIDController.setSetpoint(-89);

	}

	public void rotateLow() {
		rotatePIDController.setSetpoint(-120);

	}
}