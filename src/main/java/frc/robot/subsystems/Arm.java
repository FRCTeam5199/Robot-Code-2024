package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Main;
import frc.robot.constants.MainConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
  
	public MainConstants constants = new MainConstants();

	PIDController rotatePIDController;
	
  public Arm() {
	TalonFX krakenArmFollower;
	krakenArmFollower = new TalonFX(constants.krakenArmFollower);

	PIDInit();
  }

  public void init(){

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

  
}