// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;

// import com.revrobotics.CANSparkLowLevel;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.abstractmotorinterfaces.VortexMotorController;
// import frc.robot.constants.MainConstants;
// import com.revrobotics.CANSparkFlex;

// public class ShooterSubsystem implements Subsystem{
//   public VortexMotorController shooterMotorLeader;
//   public VortexMotorController shooterMotorFollower;
//   /** Creates a new shooter. */

//   public ShooterSubsystem() {
//     init();
// }

// public void init() {
//     motorInit();
// }

//   //one shooter (probably kraken), feeder (probably bag)
//   public void motorInit() {
//     shooterMotorLeader = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_LEADER_MOTOR_ID);
//     shooterMotorFollower = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_FOLLOWER_MOTOR_ID);
//     shooterMotorLeader.setOpenLoopRampRate(1);

//     shooterMotorFollower.follow(shooterMotorLeader.vortex, false);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   public Command shootSpeaker(){
//     return this.runOnce(() -> shooterMotorLeader.set(0));
//   }

//   public Command shootAmp(){
//     return this.runOnce(() -> shooterMotorLeader.set(0));
//   }

//   public Command shootTrap(){
//     return this.runOnce(() -> shooterMotorLeader.set(0));
//   }
// }
