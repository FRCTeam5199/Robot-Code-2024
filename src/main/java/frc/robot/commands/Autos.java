// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drivetrain.swerveDrive.SwerveDrive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.controllers.*;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.*;
import com.pathplanner.lib.util.*;

public final class Autos {
  public PathPlannerPath path;
  AutoBuilder autoBuilder;
  HolonomicPathFollowerConfig pathConfig;

  

  public Autos(SwerveDrive swerveDrive) {

    pathConfig = new HolonomicPathFollowerConfig(new PIDConstants(1, 0, 0, 0), new PIDConstants(1, 0, 0, 0), 8, .813, new ReplanningConfig(), 0);
    
    AutoBuilder.configureHolonomic(()-> swerveDrive.getPose(), (pose) -> swerveDrive.resetOdometry(pose), ()-> swerveDrive.getCurrentRobotChassisSpeeds(), (speed)-> swerveDrive.drive(speed), pathConfig, swerveDrive);
    
  }

  public Command auton1(){

    return AutoBuilder.buildAuto("1");
  }
}