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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;
import com.revrobotics.CANSparkFlex;

public class ShooterSubsystem implements Subsystem{
  public VortexMotorController shooterMotor1;
  public VortexMotorController shooterMotor2;

  public VortexMotorController shooterIndexMotor;

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
    shooterMotor1 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_LEADER_MOTOR_ID);
    shooterMotor2 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_FOLLOWER_MOTOR_ID);
    
    shooterIndexMotor = new VortexMotorController(4);
    // shooterMotorLeader.setOpenLoopRampRate(1);

    // shooterMotorFollower.follow(shooterMotorLeader.vortex, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runShooter() {
        return new InstantCommand(() -> run());
  }

  public void run() {
      System.out.println("run()");
      shooterMotor1.set(1);
      shooterMotor2.set(1);
      shooterIndexMotor.set(1);
  }

  public void changeGoal(){
    if (goalAmp) {
      goalAmp = false;
    } else {
      goalAmp = true;
    }
  }

}
