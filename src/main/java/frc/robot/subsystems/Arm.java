package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Main;
import frc.robot.constants.MainConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {

	PIDController rotatePIDController;
	TalonFX krakenArmLeader;
	TalonFX krakenArmFollower;




  public Arm() {
	krakenArmLeader = new TalonFX(MainConstants.IDs.Motors.ARM_LEADER_MOTOR_ID );
	krakenArmFollower = new TalonFX(MainConstants.IDs.Motors.ARM_FOLLOWER_MOTOR_ID);
	krakenArmFollower.setControl(new Follower(MainConstants.IDs.Motors.ARM_LEADER_MOTOR_ID,true));
  }

  public void init(){
	  PIDInit();
  }

  @Override
  public void periodic() {

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