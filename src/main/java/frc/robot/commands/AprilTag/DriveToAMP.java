// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.AprilTag;

// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.AprilTagSubsystem;

// public class DriveToAMP extends Command {

//   /** Creates a new DriveToAmp. */
//   public AprilTagSubsystem aprilTagSubsystem;
//   public PhotonTrackedTarget AMP;
//   public double rotationSpeed;
//   public PIDController turnController;


//   public DriveToAMP(AprilTagSubsystem aprilTagSubsystem) {
//       this.aprilTagSubsystem = aprilTagSubsystem;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//      turnController = new PIDController(0.1,0,0);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

// }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
