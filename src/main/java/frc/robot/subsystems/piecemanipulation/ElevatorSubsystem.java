package frc.robot.subsystems.piecemanipulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstractMotorInterfaces.SparkMotorController;
import frc.robot.constants.Constants;

public class ElevatorSubsystem extends SubsystemBase {
	SparkMotorController elevatorMotorController;
	PIDController elevatorPIDController;

	public ElevatorSubsystem() {}

	public void init() {
		motorInit();
        if (!Constants.ARM_ELEVATOR_MANUAL) {
			PIDInit();
		}

		elevatorMotorController.resetEncoder();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
        if (!Constants.ARM_ELEVATOR_MANUAL) {
			elevatorMotorController.moveAtPercent(elevatorPIDController.calculate(elevatorMotorController.getRotations()));
		}
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void motorInit() {
		elevatorMotorController = new SparkMotorController(Constants.ELEVATOR_MOTOR_ID);
        elevatorMotorController.setBrake(true);
	}

    public void PIDInit() {
		elevatorPIDController = new PIDController(Constants.ARM_ROTATE_PID.P, Constants.ARM_ROTATE_PID.I, Constants.ARM_ROTATE_PID.D);
    }

	public Command resetEncoder() {
		return this.runOnce(() -> elevatorMotorController.resetEncoder());
	}

	public Command setSetpoint(int setpoint) {
		return this.runOnce(() -> elevatorPIDController.setSetpoint(setpoint));
	}

	public void humanPlayer() {
		elevatorPIDController.setSetpoint(32);
	}

	public void low() {
		elevatorPIDController.setSetpoint(0);
	}

	public void medium() {
		elevatorPIDController.setSetpoint(4);
	}

	public void high() {
		elevatorPIDController.setSetpoint(36);
	}

	/**
	 * Moves the Elevator by a percent between -1 and 1 and stops it when finished.
	 */
	public Command move(float percent) {
		return this.runEnd(() -> elevatorMotorController.moveAtPercent(percent), () -> elevatorMotorController.moveAtPercent(0));
	}
}