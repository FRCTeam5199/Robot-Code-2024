package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Main;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;


public class ArmSubsystem extends SubsystemBase {
	private static ArmSubsystem armSubsystem;

	private static VortexMotorController armMotor;

	private PIDController rotatePIDController;
	private double rotateSetpoint = 120;

	private static boolean isBrakeMode = false; 

	private boolean armClimbMode = false;

	private CANSparkMax encoderMotor;
	private SparkAbsoluteEncoder encoder;

	public double encoderValue;

	public ArmSubsystem() {}

	/** 
	 * Gets the instnace of the Arm Subsystem.
	 */
	public static ArmSubsystem getInstance() {
		if (armSubsystem == null) {
			armSubsystem = new ArmSubsystem();
		}

		return armSubsystem;
	}

	/**
	 * init for arm and pid controller
	 */
	public void init() {
		try { motorInit(); } catch (Exception exception) {
			System.err.println("One or more issues occured while trying to initalize motors for Arm Subsystem");
			System.err.println("Exception Message:" + exception.getMessage());
			System.err.println("Exception Cause:" + exception.getCause());
			System.err.println("Exception Stack Trace:" + exception.getStackTrace()); }

	try { PIDInit(); } catch (Exception exception) {
			System.err.println("One or more issues occured while trying to initalize PID for Arm Subsystem");
			System.err.println("Exception Message:" + exception.getMessage());
			System.err.println("Exception Cause:" + exception.getCause());
			System.err.println("Exception Stack Trace:" + exception.getStackTrace()); }
	
		// Shuffleboard.getTab("Test").add("Arm Subsystem Initalized", true).getEntry();
	}

	public void motorInit() {
		armMotor = new VortexMotorController(MainConstants.IDs.Motors.ARM_MOTOR_ID);

		armMotor.setInvert(true);
		armMotor.setBrake(true);

		encoderMotor = new CANSparkMax(MainConstants.IDs.Motors.ARM_ENCODER_MOTOR, MotorType.kBrushed);
		encoder = encoderMotor.getAbsoluteEncoder(Type.kDutyCycle);
		
	}

	public void PIDInit() {
		rotatePIDController = new PIDController(MainConstants.PIDConstants.ARM_PID.P, MainConstants.PIDConstants.ARM_PID.I, MainConstants.PIDConstants.ARM_PID.D);
	}

	@Override
	public void periodic() {
		if(encoder.getPosition() < 175){
			encoderValue = encoder.getPosition();
		}

		// if (armMotor.getEncoder().getPosition() < 0) {
		// 	while (armMotor.getEncoder().getPosition() < 0) {
		// 		armMotor.set(0.3);
		// 	}
			
		// 	armMotor.set(0);
		// 	this.rotateSetpoint = 0;
		// } else if (armMotor.getEncoder().getPosition() > 61) {
		// 	while (armMotor.getEncoder().getPosition() > 61) {
		// 		armMotor.set(-0.3);
		// 	}

		// 	armMotor.set(0);
		// 	this.rotateSetpoint = 61;
		// } else {
			// if(climbModeEnabled == false) {
				armMotor.set(rotatePIDController.calculate(encoderValue, this.rotateSetpoint));
			// }
		// }
	}

	public Command toggleMode() {
    	return this.runOnce(() -> armClimbMode = !armClimbMode);
  	}

	/**
	 * True is climb
	 * @param mode
	 */
	public Command setMode(boolean mode) {
    	return this.runOnce(() -> armClimbMode = mode);
  	}
	
	/**
	 * 
	 * @param percent move armMotor at a percent(-1 to 1)
	 * @return command to spin motor at percent
	 */
	public Command moveAtPercent(double percent) {
		return this.run(() -> armMotor.set(percent));
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
    	return this.runOnce(()-> rotateSetpoint = setpoint);
 	 }	
  	/**
	 * set Setpoint variable to Stable
	 */
	public Command rotateStable() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_STABLE_SETPOINT);
	}

	public Command rotateAmp() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_AMP_SETPOINT);
	}

	/**
	 * set Setpoint variable to back
	 */
	public Command rotateBack() {
		return this.runOnce(()-> rotateSetpoint = MainConstants.Setpoints.ARM_SPEAKER_BACK_SETPOINT);
	}
	
	/**
	 * set Setpoint variable to front
	 */
	public Command rotateFront() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_SPEAKER_FRONT_SETPOINT);
	}

	/**
	 * set Setpoint variable to slightly above retracted intake
	 */
	public Command rotateIntake() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_INTAKE_SETPOINT);
	}

	/**
	 * set Setpoint variable to slightly above retracted intake
	 */
	public Command rotateClimb() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_CLIMB_SETPOINT);
	}

	public Command rotateTopPiece() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TOP_PIECE_SETPOINT);
	}

	public Command rotateBottomPiece() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_BOTTOM_PIECE_SETPOINT);
	}

	public Command rotateMiddlePiece() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_MIDDLE_PIECE_SETPOINT);
	}

	public Command rotateTrap() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TRAP_SETPOINT);
	}

	public Command rotateSubwoofer() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_SUBWOOFER_SETPOINT);
	}

	public Command rotatePodium() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_PODIUM_SETPOINT);
	}

	public Command rotateRedLine() {
		return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_RED_LINE_SETPOINT);
	}

	public Command increseAngle() {
		return this.runOnce(() -> rotateSetpoint += 1);
	}

	public Command decreseAngle() {
		return this.runOnce(() -> rotateSetpoint -= 1);
	}

	public static void toggleBrakeMode() {
        isBrakeMode = !isBrakeMode;
        armMotor.setBrake(isBrakeMode);
	}
}