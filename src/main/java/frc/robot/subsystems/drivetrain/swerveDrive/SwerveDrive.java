package frc.robot.subsystems.drivetrain.swerveDrive;

import frc.robot.Constants.AbstractConstants;

public class SwerveDrive {
    SwerveModules frontLeft;
    SwerveModules frontRight;
    SwerveModules backLeft;
    SwerveModules backRight;
    


    public SwerveDrive(){
        frontLeft = new SwerveModules(AbstractConstants.FRONT_LEFT_DRIVE_MOTOR_ID, AbstractConstants.FRONT_LEFT_STEER_MOTOR_ID, AbstractConstants.FRONT_LEFT_ENCODER, AbstractConstants.CAN_BUS, AbstractConstants.FRONT_LEFT_SWERVE_OFFSET, AbstractConstants.INVERT_FL_DRIVE, AbstractConstants.INVERT_FL_STEER);

        frontRight = new SwerveModules(AbstractConstants.FRONT_RIGHT_DRIVE_MOTOR_ID, AbstractConstants.FRONT_RIGHT_STEER_MOTOR_ID, AbstractConstants.FRONT_RIGHT_ENCODER, AbstractConstants.CAN_BUS, AbstractConstants.FRONT_RIGHT_SWERVE_OFFSET, AbstractConstants.INVERT_FR_DRIVE, AbstractConstants.INVERT_FR_STEER);

    }
}
