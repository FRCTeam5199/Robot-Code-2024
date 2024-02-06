package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.abstractMotorInterfaces.TalonMotorController;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;


public class ArmSubsystem extends SubsystemBase {
	private static ArmSubsystem armSubsystem;

	public static VortexMotorController ArmMotor;
	
	 public TalonMotorController ArmLeader;
	public TalonMotorController ArmFollower;

	public double rotateSetpoint = 0;
	PIDController rotatePIDController;

	private static boolean isBrakeMode = false; 

	boolean climbModeEnabled = false;

	public CANSparkMax encoderMotor;
	public SparkAbsoluteEncoder encoder;

	public double EncoderValue;

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
		ArmMotor = new VortexMotorController(MainConstants.IDs.Motors.ARM_MOTOR_ID);

		ArmMotor.setInvert(true);
		ArmMotor.setBrake(true);

		ArmMotor.getEncoder().setPosition(0);

		encoderMotor = new CANSparkMax(8, MotorType.kBrushed);
		encoder = encoderMotor.getAbsoluteEncoder(Type.kDutyCycle);
		
	}

	public void PIDInit() {
				rotatePIDController = new PIDController(MainConstants.PIDConstants.ARM_PID.P, MainConstants.PIDConstants.ARM_PID.I,
				MainConstants.PIDConstants.ARM_PID.D);
	}

	@Override
	public void periodic() {
		if(encoder.getPosition() <175){
			EncoderValue = encoder.getPosition();
		}

		if (ArmMotor.getEncoder().getPosition() < 0) {
			while (ArmMotor.getEncoder().getPosition() < 0) {
					ArmMotor.set(0.3);
			}
			
			ArmMotor.set(0);
			this.rotateSetpoint = 0;
		} else if (ArmMotor.getEncoder().getPosition() > 61) {
			while (ArmMotor.getEncoder().getPosition() > 61) {
				ArmMotor.set(-0.3);
			}

			ArmMotor.set(0);
			this.rotateSetpoint = 61;
		} else {
			if(climbModeEnabled == false){
				ArmMotor.set(rotatePIDController.calculate(EncoderValue, 40));
			}
		}
	}
	public Command teleOpMode(){
	return this.run(()-> new InstantCommand(){{
		climbModeEnabled = false;
    }});
	}

	public Command climbMode(){
    return this.run(()-> new Command(){{
		climbModeEnabled = true;
		if(climbModeEnabled){
      	ArmMotor.set(rotatePIDController.calculate(ArmMotor.getEncoder().getPosition(), 8));
		// 	System.out.println("climb mode "+ rotatePIDController.calculate(ArmMotor.getEncoder().getPosition(), 0.1));
		}
    }});
  }
  	public void getEncoder(){
		System.out.println(encoder.getPosition());
	}
	/**
	 * 
	 * @param percent move armMotor at a percent(-1 to 1)
	 * @return command to spin motor at percent
	 */
	public Command moveAtPercent(double percent) {
		return this.run(() -> ArmMotor.set(percent));
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
    return this.runOnce(() -> rotatePIDController.setSetpoint(setpoint));
  }
  	/**
	 * set Setpoint variable to Stable
	 */
	public void rotateStable() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_STABLE_SETPOINT;
	}

	/**
	 * set Setpoint variable to back
	 */
	public void rotateBack() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_SPEAKER_BACK_SETPOINT;
	}
	
	/**
	 * set Setpoint variable to front
	 */
	public void rotateFront() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_SPEAKER_FRONT_SETPOINT;
	}

	/**
	 * set Setpoint variable to slightly above retracted intake
	 */
	public void rotateIntake() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_INTAKE_SETPOINT;
	}

	/**
	 * set Setpoint variable to slightly above retracted intake
	 */
	public void rotateClimb() {
		this.rotateSetpoint = MainConstants.Setpoints.ARM_CLIMBER_SETPOINT;
		
	}
	public static void toggleBrakeMode() {
        isBrakeMode = !isBrakeMode;
        ArmMotor.setBrake(isBrakeMode);
    }

}