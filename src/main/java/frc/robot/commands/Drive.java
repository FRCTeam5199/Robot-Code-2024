package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.swerveDrive.SwerveDrive;

public class Drive extends Command {
    SwerveDrive swerveDrive = new SwerveDrive();
    ChassisSpeeds drive;
    
    public Drive(double x, double y, double rotate){
        drive = new ChassisSpeeds(x, y, rotate);
        swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotate, new Rotation2d(0)));

        addRequirements(swerveDrive);
    }
}
