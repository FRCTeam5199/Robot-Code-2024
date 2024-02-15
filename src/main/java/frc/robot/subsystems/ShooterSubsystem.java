// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

public class
ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem shooterSubsystem;
  
  private static boolean subsystemStatus = false;

  public VortexMotorController shooterMotor1;
  public VortexMotorController shooterMotor2;

  public VortexMotorController shooterIndexerMotor;

  public CANSparkMax shooterFlippyDoMotor;
  public SparkPIDController shooterFlippyDoPIDConroller;

  public double shooterSpeed;
  public double indexerSpeed;

  public boolean ampAndClimbMode = false;
  public boolean runShooter = false;
  public boolean runIndexer = false;
  public boolean intakeShooter = false;
  public boolean shoot = false;
  public boolean autonSide = false;

  public GenericHID genericHID = new GenericHID(0);

  public ShooterSubsystem() {}
  
	/** 
	 * Gets the instance of the Shooter Subsystem.
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
  }

  public boolean getSubsystemStatus() {
    return subsystemStatus;
  }

  /**
   *  Initalizes the motor(s) for this subsystem
   */
  public void motorInit() {
    shooterMotor1 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_1_ID);
    shooterMotor1.getEncoder().setPosition(0);
    shooterMotor1.setInvert(true);

    shooterMotor2 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_2_ID);
    shooterMotor2.getEncoder().setPosition(0);
    shooterMotor2.setInvert(false);

    shooterIndexerMotor = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_INDEXER_MOTOR_ID);    
    shooterIndexerMotor.setInvert(true);
    shooterIndexerMotor.setBrake(true);

    shooterFlippyDoMotor = new CANSparkMax(11, MotorType.kBrushless);
  }

  public void PIDInit() {
    shooterFlippyDoPIDConroller = shooterFlippyDoMotor.getPIDController();
    shooterFlippyDoPIDConroller.setP(0.1);
  }

  public RelativeEncoder getShooterMotor1Encoder() {
		return shooterMotor1.getEncoder();
	}

  public RelativeEncoder getShooterMotor2Encoder() {
		return shooterMotor2.getEncoder();
	}

  public RelativeEncoder getShooterIndexerMotorEncoder() {
		return shooterIndexerMotor.getEncoder();
	}

  public RelativeEncoder getShooterFlippyDoMotorEncoder() {
		return shooterFlippyDoMotor.getEncoder();
	}
  
  public boolean checkMotors() {
    if (shooterMotor1 != null && shooterMotor2 != null) {
      return true;
    } else {
      subsystemStatus = false;
      return false;
    }
  }

  public boolean checkPID() {
    if (shooterFlippyDoPIDConroller != null) {
      return true;
    } else {
      subsystemStatus = false;
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (checkMotors()) { subsystemStatus = true; } else { subsystemStatus = false; }

    if (subsystemStatus) {
      subsystemPeriodic();
    }
  }
  
  private void subsystemPeriodic() {
    if (checkForGamePiece()) {
      genericHID.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    } else {
      genericHID.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    if (ampAndClimbMode) {
      shooterSpeed = 0.3;
      indexerSpeed = 0.5;
    } else if (intakeShooter) {
      shooterSpeed = -1;
      indexerSpeed = -0.5;
    } else {
      shooterSpeed = 0.75;
      indexerSpeed = 0.5;
    }
    
    if(autonSide){
      shooterSpeed = 0.5;
    }

    if(intakeShooter){
      shooterMotor1.set(-.3);
      shooterMotor2.set(-.3);
    } else {
      if (runShooter) {
        // if (ampAndClimbMode == false) {
        shooterMotor1.set(shooterSpeed);
        // }
        shooterMotor2.set(shooterSpeed);
      } else {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
      }
    }
    if(intakeShooter) {
      shooterIndexerMotor.set(-.3);
    } else {
      if (runIndexer) {
        shooterIndexerMotor.set(indexerSpeed);
      } else {
        shooterIndexerMotor.set(0);
      }
    }
  }

  public Command flippyDoSetpoint(double Setpoint){
    return this.runOnce(()-> shooterFlippyDoPIDConroller.setReference(Setpoint, ControlType.kPosition));
  }

  public Command setRunShooter(boolean runShooter) {
    return this.runOnce(() -> this.runShooter = runShooter);
  }

  public Command setRunIndexer(boolean runIndexer) {
    return this.runOnce(() -> this.runIndexer = runIndexer);
  }

  public Command setAmpandClimbMode(boolean ampAndClimbMode) {
    return this.runOnce(() -> this.ampAndClimbMode = ampAndClimbMode);
  }

  public Command runAutonShooting(boolean side) {
    return new SequentialCommandGroup(
      autonSide(side),
      setRunShooter(true), new WaitCommand(.5), setRunIndexer(true),
    new WaitCommand(0.3), setRunShooter(false), setRunIndexer(false), autonSide(false));
  }

  public Command autonSide(boolean side){
    return this.runOnce(()-> this.autonSide = side);
  }

   /**
   * Runs the Shooter Motor to Intake.
   * True runs the Shooter to Intake.
   * @param intakeShooter
   */
  public Command setIntakeShooter(boolean intakeShooter) {
    return this.runOnce(() ->  this.intakeShooter = intakeShooter);
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
