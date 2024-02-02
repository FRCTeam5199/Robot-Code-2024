package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Main;
import frc.robot.abstractMotorInterfaces.TalonMotorController;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;


public class ArmSubsystem extends SubsystemBase {
	public VortexMotorController ArmMotor;

	public CANSparkMax sparkMax;
	public RelativeEncoder encoder;	
	
	public double rotateSetpoint = 0;
	PIDController rotatePIDController;

	AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
	double shooterAngle;
	double rotateDegrees;
	double angleSetpoint = 0;

	public ArmSubsystem() {}

	public void init() {
		motorInit();
		PIDInit();
		encoder.setInverted(true);
	}

	public void motorInit() {
		ArmMotor = new VortexMotorController(MainConstants.IDs.Motors.ARM_MOTOR_ID);

		ArmMotor.setInvert(true);
		ArmMotor.setBrake(true); 
		//Make true and setpoint positive and zero arm at the top for working

		ArmMotor.getEncoder().setPosition(0);

		sparkMax = new CANSparkMax(8, MotorType.kBrushed);
		encoder = sparkMax.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 8092);	
		encoder.setPosition(0);
	}

	public void PIDInit() {
				rotatePIDController = new PIDController(MainConstants.PIDConstants.ARM_PID.P, MainConstants.PIDConstants.ARM_PID.I,
				MainConstants.PIDConstants.ARM_PID.D);
	}

	@Override
	public void periodic() {
		// if (encoder.getPosition() < 2) {
		// 	while (encoder.getPosition() < 2) {
		// 			ArmMotor.set(0.3);
		// 	}
			
		// 	ArmMotor.set(0);
		// 	this.rotateSetpoint = encoder.getPosition();
		// } else if (encoder.getPosition() > 61) { //change this
		// 	while (encoder.getPosition() > 61) { //change this
		// 		ArmMotor.set(-0.3);
		// 	}

		// 	ArmMotor.set(0);
		// 	this.rotateSetpoint = encoder.getPosition();
		// } else {
		// 		ArmMotor.set(rotatePIDController.calculate(encoder.getPosition(), rotateSetpoint));
		// }
		
		// ArmMotor.set(rotatePIDController.calculate(encoder.getPosition(), -30));
		
		// rotatePIDController.calculate(encoder.getPosition(), -0.176025390625);
		// System.out.println(-rotatePIDController.calculate(encoder.getPosition(), -0.08835887163877487));
		// ArmMotor.set((-rotatePIDController.calculate(encoder.getPosition(), -0.08835887163877487))/MainConstants.ROTATIONS_PER_1_DEGREE_ARM);// -0.05598121136426926)));
		// ArmMotor.set(rotatePIDController.calculate(ArmMotor.getEncoder().getPosition(), rotateSetpoint));
		ArmMotor.set(-rotatePIDController.calculate(encoder.getPosition(), angleSetpoint/360)/MainConstants.ROTATIONS_PER_1_DEGREE_ARM);
	}
	
	/**
	 * 
	 * @param percent move armMotor at a percent(-1 to 1)
	 * @return command to spin motor at percent
	 */
	public Command moveAtPercent(double percent) {
		return this.runOnce(() -> ArmMotor.set(this.rotateSetpoint));
	}

	public Command testEncoder() {
		return this.runOnce(() -> System.out.println("pos //////////////////"+ encoder.getPosition()));
	}
	
	public Command getSpeedOfArm(){
		return this.runOnce(()-> System.out.println(-rotatePIDController.calculate(encoder.getPosition(), angleSetpoint/360)/MainConstants.ROTATIONS_PER_1_DEGREE_ARM));
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

  public Command moveToAngle() {
	return this.run(() -> calculateSetpointBasedOnAngle());
  }



  public void calculateSetpointBasedOnAngle() {
	angleSetpoint = -1 * (aprilTagSubsystem.speakersAligning() - MainConstants.ARM_ORIGINAL_DEGREES);
	// System.out.println(angleSetpoint);
	
  }
  


  public Command setAngleSetPointZero() {
		return this.run(()-> this.angleSetpoint = 0);
	}
  	/**
	 * set Setpoint variable to Stable
	 */
	public Command rotateStable() {
		return this.run(()-> this.angleSetpoint = -32.8);
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
	public Command rotateFront() {
		// this.rotateSetpoint = MainConstants.Setpoints.ARM_SPEAKER_FRONT_SETPOINT;
		// this.angleSetpoint = 60.5;
		return this.run(()-> this.angleSetpoint = -60.5);
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
}
