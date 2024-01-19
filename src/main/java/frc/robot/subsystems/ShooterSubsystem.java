// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;
import com.revrobotics.CANSparkFlex;

public class ShooterSubsystem implements Subsystem{

  public MainConstants constants = new MainConstants();
  TalonFX krakenShooter;
  CANSparkFlex shooter;
  /** Creates a new shooter. */

  //one shooter (probably kraken), feeder (probably bag)
  public ShooterSubsystem() {
    shooter = new CANSparkFlex(1, CANSparkLowLevel.MotorType.kBrushless);
    shooter.setOpenLoopRampRate(5);
  }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
