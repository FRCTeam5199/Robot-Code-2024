// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;
import com.revrobotics.CANSparkFlex;

public class ShooterSubsystem implements Subsystem{
  public VortexMotorController shooterMotor1;
  public VortexMotorController shooterMotor2;

  public VortexMotorController shooterIndexerMotor;

  private boolean goalAmp = false;
  /** Creates a new shooter. */

  public ShooterSubsystem() {
    init();
}

public void init() {
    motorInit();
}

  //one shooter (probably kraken), feeder (probably bag)
  public void motorInit() {
    shooterMotor1 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_1_ID);
    shooterMotor2 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_2_ID);
    shooterIndexerMotor = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_INDEXER_MOTOR_ID);

    shooterMotor1.setInvert(true);
    shooterMotor2.setInvert(false);

    shooterIndexerMotor.setBrake(true);
    // shooterMotorLeader.setOpenLoopRampRate(1);

    // shooterMotorFollower.follow(shooterMotorLeader.vortex, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command setIndexerSpeed(double percent) {
    return this.runOnce(() -> shooterIndexerMotor.set(percent));
  }
  
  public Command setShooterSpeed(double percent) {
    return this.runOnce(() -> shooterMotor1.set(percent)).andThen(() -> shooterMotor2.set(percent));
  }

  public Command intakeShooter() {
    return this.runOnce(() -> shooterIndexerMotor.set(-0.3)).alongWith(
      new InstantCommand(() -> shooterMotor1.set(-0.3)),
      new InstantCommand(() -> shooterMotor2.set(-0.3)));
  }

  public Command stopShooter() {
    return this.runOnce(() -> shooterIndexerMotor.set(0)).alongWith(
      new InstantCommand(() -> shooterMotor1.set(0)),
      new InstantCommand(() -> shooterMotor2.set(0)));
  }
  
  public boolean checkForGamePiece(){
    if (shooterIndexerMotor.getVelocity() < 5){
      return true;
    } 
    return false;
  }

}
