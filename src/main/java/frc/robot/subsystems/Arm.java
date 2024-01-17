package frc.robot.subsystems;

import com.frc.robot.Constants;
import com.frc.robot.abstractMotorInterfaces.TalonMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
  
  public TalonMotorController rotateMotorController;
	PIDController rotatePIDController;
	

  public Arm() {
  }

  public void init(){
    motorinit();
    PIDinit();

		rotateMotorController.resetEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rotateMotorController.moveAtPercent(rotatePIDController.calculate(rotateMotorController.getRotations()));
  }

	public void motorInit() {
		rotateMotorController = new TalonMotorController(Constants.ARM_ROTATE_MOTOR_ID, MotorType.kBrushless);

		rotateMotorController.setBrake(true);
	}

  public void PIDInit() {
		rotatePIDController = new PIDController(Constants.ARM_ROTATE_PID.P, Constants.ARM_ROTATE_PID.I,
				Constants.ARM_ROTATE_PID.D);
	}

	public Command resetRotateEncoder() {
		return this.runOnce(() -> rotateMotorController.resetEncoder());
	}

	public Command setRotateSetpoint(int setpoint) {
		return this.runOnce(() -> rotatePIDController.setSetpoint(setpoint));
	}

  public Command moveRotate(float percent) {
		return this.runEnd(() -> rotateMotorController.moveAtPercent(percent),
				() -> rotateMotorController.moveAtPercent(0));
  }
  
}