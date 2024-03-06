// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.abstractMotorInterfaces.TalonMotorController;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem shooterSubsystem;
    public SwerveDrive drive = TunerConstants.DriveTrain;

    public VortexMotorController shooterMotor1;
    public VortexMotorController shooterMotor2;

    public TalonMotorController shooterTop;
    public TalonMotorController shooterBottom;

    public SparkPIDController shooterMotor1PidController;

    public double shooterSpeed;

    public double shooterSpeedOffset;

    public double setRPM = 0;
    public double speedPercent = 0;

    public PIDController topWheelPIDController;
    public PIDController bottomWheelPIDController;


    public boolean autoTargeting = true;
    public boolean ampMode = false;
    public boolean runShooter = false;
    public boolean runIndexer = false;
    public boolean intakeShooter = false;
    public boolean autonSide = false;
    public boolean idleShooting = false;

    public ShooterSubsystem() {
    }

    /**
     * Gets the instnace of the Arm Subsystem.
     */
    public static ShooterSubsystem getInstance() {
        if (shooterSubsystem == null) {
            shooterSubsystem = new ShooterSubsystem();
        }

        return shooterSubsystem;
    }

    public void init() {
        try {
            motorInit();
        } catch (Exception exception) {
            System.err.println("One or more issues occured while trying to initalize motors for Shooter Subsystem");
            System.err.println("Exception Message:" + exception.getMessage());
            System.err.println("Exception Cause:" + exception.getCause());
            System.err.println("Exception Stack Trace:" + exception.getStackTrace());
        }
    }

    /*
     * Initalizes the motor(s) for this subsystem
     */
    public void motorInit() {
        shooterMotor1 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_1_ID, 0.0008443300, 0.000001, 200, 0.0001929);
        shooterMotor2 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_2_ID, 0.000812973, 0.000001, 200, 0.0001892);


        shooterMotor1.setInvert(true);
        shooterMotor2.setInvert(false);


        shooterMotor1.getEncoder().setPosition(0);
        shooterMotor2.getEncoder().setPosition(0);

        shooterMotor1.setCurrentLimit(30);
        shooterMotor2.setCurrentLimit(30);
    }

    @Override
    public void periodic() {
        

    }

    public double autoSpeed() {
        return  (drive.getPose().getTranslation().getDistance(new Translation2d(16.579342, 5.547)) - 1.27) * (6800 - 3000) / (5.7912 - 1.27) + 3000;
         
    }

    public double autonShoot() {
        return (drive.getPose().getTranslation().getDistance(new Translation2d(16.579342, 5.547)) - 1.27) * (6800 - 3000) / (5.7912 - 1.27) + 3000;
        //double x, double in_min, double in_max, double out_min, double out_max
        // return (x- in_min) * (out_max - out_min) / (in_max -in_min) + out_min;
    }

    public void IdleRevUp() {
        setRPM = 2000;
        runShooter = true;
    }


    public Command setAutoTargeting(boolean bool){
        return this.runOnce(()-> autoTargeting = bool);
    }
    public Command autoAimShooting(){
        return this.run(()->runShooterAtRpm(autoSpeed()).andThen(()-> System.out.println(autoSpeed())));
    }

    public Command runShooterAtRpm(double vel) {
        return this.runOnce(() -> shooterMotor1.setVelocity(vel + shooterSpeedOffset)).andThen(() -> shooterMotor2.setVelocity(vel + shooterSpeedOffset));
    }

    public Command runShooterAtPercent(double per) {
        return this.runOnce(() -> shooterMotor1.set(per)).andThen(() -> shooterMotor2.set(per));
    }

    public Command runShooterPredeterminedRPM() {
        return this.runOnce(() -> shooterMotor1.setVelocity(setRPM + shooterSpeedOffset)).andThen(() -> shooterMotor2.setVelocity(setRPM + shooterSpeedOffset));
    }

    public Command runShooterClimbAmp(double vel) {
        return this.runOnce(() -> shooterMotor1.setVelocity(vel + shooterSpeedOffset)).andThen(() -> shooterMotor2.set(0));
    }

    public Command increaseShooterSpeed() {
        return this.runOnce(() -> shooterSpeedOffset += 50).andThen(() -> System.out.println(shooterSpeedOffset));
    }

    public Command decreaseShooterSpeed() {
        return this.runOnce(() -> shooterSpeedOffset -= 50).andThen(() -> System.out.println(shooterSpeedOffset));
    }

    public Command setRunShooter(boolean runShooter) {
        return this.runOnce(() -> this.runShooter = runShooter);
    }

    public Command setAmpMode(boolean ampMode) {
        return this.runOnce(() -> this.ampMode = ampMode);
    }

    public Command autonSpeed(boolean side) {
        return this.runOnce(() -> this.autonSide = side);
    }

    /**
     * Runs the Shooter Motor to Intake
     */
    public Command setintakeShooter(boolean intakeShooter) {
        return this.runOnce(() -> this.intakeShooter = intakeShooter);
    }

    /**
     * @param sp rpm you want the shooter to be at
     * @return sets the shooter rpm.
     */
    public Command setRPMShooter(double sp) {
        return this.runOnce(() -> this.setRPM = sp);
    }

    public double getRPMShooter() {
        return setRPM;
    }

   
    /**
     * Sets the Shooter motor speed to a percent between -1 and 1
     *
     * @param
     */
    public Command setShooterSpeed(double percent) {
        return this.runOnce(() -> speedPercent = percent);
    }

    /**
     * Sets the Shooter motor velocity based on the RPM of the motor
     *
     * @param
     */
    public Command setShooterVelocity(double velocity) {
        return this.runOnce(() -> runShooting(velocity));
    }

    private void runShooting(double v) {
        shooterMotor1.setVelocity(v);
        shooterMotor2.setVelocity(v);
    }



    public boolean reachedSpeed() {
        if (setRPM > 0) {
            if (shooterMotor1.getVelocity() >= setRPM - 1 && shooterMotor2.getVelocity() >= setRPM - 1) {
                return true;
            }
        }
        return false;
    }
}


    


   