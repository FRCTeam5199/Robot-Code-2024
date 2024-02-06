package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem intakeSubsystem;

    public VortexMotorController intakeMotor;
    public VortexMotorController intakeActuatorMotor;
    public PIDController pidController;
    public double setpoint;
    public SparkPIDController sparkPIDController;

    public IntakeSubsystem() {}

	/** 
	 * Gets the instnace of the Arm Subsystem.
	 */
	public static IntakeSubsystem getInstance() {
		if (intakeSubsystem == null) {
			intakeSubsystem = new IntakeSubsystem();
		}

		return intakeSubsystem;
	}

    public void init() {
        try { motorInit(); } catch (Exception exception) {
            System.err.println("One or more issues occured while trying to initalize motors for Intake Subsystem");
            System.err.println("Exception Message:" + exception.getMessage());
            System.err.println("Exception Cause:" + exception.getCause());
            System.err.println("Exception Stack Trace:" + exception.getStackTrace()); }

        try { PIDInit(); } catch (Exception exception) {
            System.err.println("One or more issues occured while trying to initalize PID for Intake Subsystem");
            System.err.println("Exception Message:" + exception.getMessage());
            System.err.println("Exception Cause:" + exception.getCause());
            System.err.println("Exception Stack Trace:" + exception.getStackTrace()); }
        
        // Shuffleboard.getTab("Test").add("Intake Subsystem Initalized", true).getEntry();

    }

    @Override
    public void periodic() {
        // intakeActuatorMotor.set(pidController.calculate(intakeActuatorMotor.getRotations(), setpoint));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * motor init of intake subsystem
     */
    public void motorInit() {
        intakeMotor = new VortexMotorController(MainConstants.IDs.Motors.INTAKE_MOTOR_ID);
        intakeActuatorMotor = new VortexMotorController(MainConstants.IDs.Motors.INTAKE_ACTUATOR_MOTOR_ID);
        
        intakeMotor.setInvert(true);
        intakeActuatorMotor.setInvert(false);

        intakeMotor.getEncoder().setPosition(0);
        intakeActuatorMotor.getEncoder().setPosition(0);
    }

    /**
     * init of the pidController
     */
    public void PIDInit() {
        // pidController = new PIDController(MainConstants.PIDConstants.INTAKE_PID.P, MainConstants.PIDConstants.INTAKE_PID.I, MainConstants.PIDConstants.INTAKE_PID.D);
        sparkPIDController = intakeActuatorMotor.vortex.getPIDController();
        sparkPIDController.setP(MainConstants.PIDConstants.ARM_PID.P);
        sparkPIDController.setI(MainConstants.PIDConstants.ARM_PID.I);
        sparkPIDController.setD(MainConstants.PIDConstants.ARM_PID.D);
    }

    /**
     * 
     * @param percent speed to set intake to
     * @return command to spin intake
     */
    public Command setIntakeSpeed(double percent) {
        return this.runOnce(() -> intakeMotor.set(percent));
    }

    public Command setIntakeActuatorSpeed(double percent) {
        return this.runOnce(() -> intakeActuatorMotor.set(percent));
    }
    
    public Command setIntakeActuatorTarget(double target) {
        return this.runOnce(() -> setpoint = target);
    }

    // TODO: Swap deploy and stow intake set positions after absolute encoder gets finalized

    /**
     * retract the intake
     * @return command to retract intake
     */
    public Command stowIntake() {
        return this.runOnce(() -> sparkPIDController.setReference(MainConstants.Setpoints.STOW_INTAKE, ControlType.kPosition));
    }

    /**
     * deploy intake
     * @return command to deploy the intake
     */
    public Command deployIntake() {
        return this.runOnce(() -> sparkPIDController.setReference(MainConstants.Setpoints.DEPLOY_INTAKE, ControlType.kPosition));
    }
}
