// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

public class ShooterSubsystem implements Subsystem{
  public VortexMotorController shooterMotor1;
  public VortexMotorController shooterMotor2;

  public VortexMotorController shooterIndexerMotor;

  public IntakeSubsystem intakeSubsystem;
  public ArmSubsystem armSubsystem;

  

  public ShooterSubsystem(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;

  }


  public void init() {
      motorInit();

      Shuffleboard.getTab("Status").add("Shooter Subsystem Status", true).getEntry();

      
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
    return this.runOnce(() -> shooterMotor1.set(percent)).andThen(() -> shooterMotor2.set(percent));
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
    return this.runOnce(() -> shooterIndexerMotor.set(-0.3)).alongWith(
      new InstantCommand(() -> shooterMotor1.set(-0.3)),
      new InstantCommand(() -> shooterMotor2.set(-0.3)));
  }
  public Command runIndexerTest(Double speed){
    return this.runOnce(()-> shooterIndexerMotor.set(speed));
  }

  /**
   * Stops the Shooter Motor
   */
  public Command stopShooter() {
    return this.runOnce(() -> shooterIndexerMotor.set(0)).alongWith(
      new InstantCommand(() -> shooterMotor1.set(0)),
      new InstantCommand(() -> shooterMotor2.set(0)));
  }
  
  public Command checkCurrent(){
    return this.runOnce(()-> System.out.println(checkForGamePiece()));
  }

  public void stopIntakeAction(){
    stopShooter();
    armSubsystem.rotateStable();
    new WaitCommand(0.4);
    intakeSubsystem.retractAndStopIntake();
    
  }

  /**
   * Checks for current spike inside of the indexer
   * @return True if a game piece is in the Indexer
   */
  private boolean currentCheck(){
    new WaitCommand(0.4);
    if (shooterIndexerMotor.getCurrent() < 55){
      return false;
    } 
    new WaitCommand(0.5);
    if (shooterIndexerMotor.getCurrent() > 55){
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
    if(shooterIndexerMotor.getCurrent()>55){
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

  public void gamePieceDected(){
    if(checkForGamePiece()){
      //stop spin bottom intake and shooter, retract bottom intake
      stopIntakeAction();

    }
    
  }

}
