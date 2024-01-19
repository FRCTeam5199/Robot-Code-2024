package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.abstractMotorInterfaces.TalonMotorController;
import frc.robot.constants.MainConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;


public class ArmSubsystem implements Subsystem {
	TalonMotorController krakenArmLeader;
	TalonMotorController krakenArmFollower;
	PIDController rotatePIDController;
	boolean isFront;

	/** Creates a new Arm. */
  public ArmSubsystem() {
	krakenArmLeader = new TalonMotorController(MainConstants.IDs.Motors.ARM_LEADER_MOTOR_ID );
	krakenArmFollower = new TalonMotorController(MainConstants.IDs.Motors.ARM_FOLLOWER_MOTOR_ID);
	krakenArmFollower.follow(krakenArmLeader, true);
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

	public void rotateHumanPlayer() {
		rotatePIDController.setSetpoint(-110);

	}

	public void rotateStable() {
		rotatePIDController.setSetpoint(-89);

	}

	public void rotateLow() {
		rotatePIDController.setSetpoint(-120);

	}

	public boolean isFront() {
	  isFront = true;
	  return true;
	}
}