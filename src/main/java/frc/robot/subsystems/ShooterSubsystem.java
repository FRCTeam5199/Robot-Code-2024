// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem shooterSubsystem;

  public VortexMotorController shooterMotor1;
  public VortexMotorController shooterMotor2;

  public VortexMotorController shooterIndexerMotor;

  private double shooterSpeed;

  public GenericHID genericHID = new GenericHID(0);

  public ShooterSubsystem() {}
  
	/** 
	 * Gets the instnace of the Arm Subsystem.
	 */
	public static ShooterSubsystem getInstance() {
		if (shooterSubsystem == null) {
			shooterSubsystem = new ShooterSubsystem();
		}

		return shooterSubsystem;
	}
  
  public void init() {
        try { motorInit(); } catch (Exception exception) {
            System.err.println("One or more issues occured while trying to initalize motors for Shooter Subsystem");
            System.err.println("Exception Message:" + exception.getMessage());
            System.err.println("Exception Cause:" + exception.getCause());
            System.err.println("Exception Stack Trace:" + exception.getStackTrace()); }

      // Shuffleboard.getTab("Test").add("Shooter Subsystem Initalized", true).getEntry();      
  }

  /*
   * Initalizes the motor(s) for this subsystem
   */
  public void motorInit() {
    shooterMotor1 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_1_ID);
    shooterMotor2 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_2_ID);
    shooterIndexerMotor = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_INDEXER_MOTOR_ID);

    shooterMotor1.setInvert(true);
    shooterMotor2.setInvert(false);
    
    shooterIndexerMotor.setInvert(true);
    shooterIndexerMotor.setBrake(true);

    shooterMotor1.getEncoder().setPosition(0);
    shooterMotor2.getEncoder().setPosition(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (checkForGamePiece()) {
      genericHID.setRumble(RumbleType.kBothRumble, 1);
    } else {
      genericHID.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  public Command runShooter() {
    return this.runOnce(() -> shooterMotor1.set(shooterSpeed)).andThen(() -> shooterMotor2.set(shooterSpeed));
  }
  
  public Command runBottomShooter() {
    return this.runOnce(() -> shooterMotor2.set(shooterSpeed));
  }

  public Command stopShooter() {
    return this.runOnce(() -> shooterIndexerMotor.set(0))
    .andThen(setShooterSpeed(0))
    .andThen(runShooter());
  }
  
  /**
   * Sets the Indexer motor speed to a percent between -1 and 1
   * @param
   */
  public Command setIndexerSpeed(double percent) {
    return this.runOnce(() -> shooterIndexerMotor.set(percent));
  }

  /**
   * Sets the Shooter motor speed to a percent between -1 and 1
   * @param
   */
  public Command setShooterSpeed(double percent) {
    return this.runOnce(() -> shooterSpeed = percent);
  }

  /**
   * Sets the Shooter motor velocity based on the RPM of the motor
   * @param
   */
  public Command setShooterVelocity(double velocity) {
    return this.runOnce(() -> shooterMotor1.setVelocity(velocity)).andThen(() -> shooterMotor2.setVelocity(velocity));
  }

  /**
   * Runs the Shooter Motor to Intake
   */
  public Command intakeShooter() {
    return this.runOnce(() -> new SequentialCommandGroup(
      setShooterSpeed(-0.3),
      runShooter(),
      new InstantCommand(() -> shooterIndexerMotor.set(-0.3))));
  }

  /**
   * Stops the Shooter Motor
   */

  /**
   * Checks for current spike inside of the indexer
   * @return True if a game piece is in the Indexer
   */
  private boolean currentCheck(){
    new WaitCommand(0.4);
    if (shooterIndexerMotor.getCurrent() < 65){
      return false;
    } 
    new WaitCommand(0.5);
    if (shooterIndexerMotor.getCurrent() > 65){
      return true;
    } 
    return false;    
  }

  /**
   * decides if a game piece is truly inside of 
   * @return true if game piece false if not
   */
  public boolean checkForGamePiece(){
    int piece = 0;
    int noPiece = 0;
    if(shooterIndexerMotor.getCurrent() > 65){
      for(int i = 0; i <=10; i++){
        if(currentCheck() == true){
          piece++;
        }
        else{
          noPiece++;
        }
      }
    }
    if(piece > noPiece){
      return true;
    }
    return false;     
  }
}
