package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

 public class ClimberSubsystem implements Subsystem {
   public VortexMotorController climberMotor1;
   public VortexMotorController climberMotor2;

   public PIDController climberPIDController;

  public ClimberSubsystem() {}

  /**
   * init for climber
   */
  public void init() {
    motorInit();
    PIDInit();

    Shuffleboard.getTab("Status").add("Climber Subsystem Status", true).getEntry();
  }

  /**
   * init for motor climbers 
   */
  public void motorInit() {
    climberMotor1 = new VortexMotorController(MainConstants.IDs.Motors.CLIMBER_MOTOR_1_ID);
    // climberMotor2 = new VortexMotorController(MainConstants.IDs.Motors.CLIMBER_CLAW_MOTOR_ID);

    climberMotor1.setInvert(false);
    // climberMotor2.setInvert(true);

    climberMotor1.getEncoder().setPosition(0);
    // climberMotor2.getEncoder().setPosition(0);
  }
  private void PIDInit() {
    climberPIDController = new PIDController(MainConstants.PIDConstants.CLIMBER_PID.P, MainConstants.PIDConstants.CLIMBER_PID.I, MainConstants.PIDConstants.CLIMBER_PID.D);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // climberMotor1.set(climberPIDController.calculate(climberMotor1.getRotations()));
    // climberMotor2.set(climberPIDController.calculate(climberMotor2.getRotations()));
  }

   @Override
   public void simulationPeriodic() {
      //This method will be called once per scheduler run during simulation
   }

  /**
   * 
   * @param percent sets climber speed 
   * @return command to set climber speed
   */
  public Command setClimberSpeed(double percent) {
    return this.runOnce(() -> climberMotor1.set(percent))/*.andThen((() -> climberMotor2.set(percent)))*/;
  }

  /**
   * 
   * @param target position where motor needs to go to 
   * @return command that makes motor go to a setPositions 
   */
  public Command setClimberTarget(double target) {
    return this.runOnce(() -> climberPIDController.setSetpoint(target));
  }

  public Command extendClimber() {
    return this.runOnce(() -> setClimberSpeed(0.5));//new SequentialCommandGroup(
      // new InstantCommand(() -> setClimberSpeed(0.5)),
      // new WaitCommand(2),
      // new InstantCommand(() -> setClimberSpeed(0))
    // );
  }

  public Command retractClimber() {
    return this.runOnce(() -> setClimberSpeed(0));//new SequentialCommandGroup(
    //   new InstantCommand(() -> setClimberSpeed(-0.5)),
    //   new WaitCommand(2),
    //   new InstantCommand(() -> setClimberSpeed(0))
    // );
  }
}