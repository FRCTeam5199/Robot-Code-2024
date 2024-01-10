// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;

public class ShooterSubsystem extends SubsystemBase {

  public MainConstants constants = new MainConstants();
  TalonFX krakenShooter;
  /** Creates a new shooter. */

  //one shooter (probably kraken), feeder (probably bag)
  public ShooterSubsystem() {
    
    krakenShooter = new TalonFX(constants.krakenShooter);
    
  }

  public Command setSpeedShooter(double speed){
    return this.run(()-> krakenShooter.set(speed));
  }  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
