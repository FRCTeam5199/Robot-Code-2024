// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utility.PID;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem shooterSubsystem;
    public SwerveDrive drive = TunerConstants.DriveTrain;

    public VortexMotorController shooterMotor1;
    public VortexMotorController shooterMotor2;

    public VortexMotorController shooterIndexerMotor;

    public SparkPIDController shooterMotor1PidController;

    public double shooterSpeed;
    public double indexerSpeed;
    public double shooterSpeedOffset;

    public double setRPM = 0;
    public double speedPercent = 0;

    public PIDController topWheelPIDController;
    public PIDController bottomWheelPIDController;


    public boolean autoTargeting = false;
    public boolean ampAndClimbMode = false;
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
      shooterMotor1 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_1_ID, 0.0008443300, 0.000001, 200,0.0001929);
      shooterMotor2 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_2_ID, 0.000812973, 0.000001, 200,0.0001892);

      shooterIndexerMotor = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_INDEXER_MOTOR_ID);

      shooterMotor1.setInvert(true);
      shooterMotor2.setInvert(false);

      shooterIndexerMotor.setInvert(false);
      shooterIndexerMotor.setBrake(true);

      shooterMotor1.getEncoder().setPosition(0);
      shooterMotor2.getEncoder().setPosition(0);

      shooterMotor1.setCurrentLimit(30);
      shooterMotor2.setCurrentLimit(30);
      shooterIndexerMotor.setCurrentLimit(40);
    }

    @Override
    public void periodic() {
      // System.out.println("amongus " + setRPM + "runShooterAmongus " + runShooter);
      if (autoTargeting){
        // System.out.println(autoSpeed(drive.getPose().getTranslation().getDistance(new Translation2d(16.579342, 5.547)), 1.27, 5.7912, 3000, 6800));
        autoSpeed();
        shooterMotor1.setVelocity(setRPM + shooterSpeedOffset);
        shooterMotor2.setVelocity(setRPM + shooterSpeedOffset);
        // System.out.println("auto shooter 1 " +shooterMotor1.getVelocity());
        // System.out.println("auto shooter 2 " +shooterMotor2.getVelocity());
        // shooterMotor1.setVelocity(autoSpeed(drive.getPose().getTranslation().getDistance(new Translation2d(16.579342, 5.547)), 1.27, 5.7912, 3000, 6800));
        // shooterMotor2.setVelocity(autoSpeed(drive.getPose().getTranslation().getDistance(new Translation2d(16.579342, 5.547)), 1.27, 5.7912, 3000, 6800));

      }
      else if (runShooter) {
        if (intakeShooter) {
          shooterMotor1.set(-0.3);
          shooterMotor2.set(-0.3);
        }
        else{
        shooterMotor1.setVelocity(setRPM + shooterSpeedOffset);
        if(ampAndClimbMode == false){
          shooterMotor2.setVelocity(setRPM + shooterSpeedOffset);
            }
        // System.out.println("shooter 1 " +shooterMotor1.getVelocity());
        // System.out.println("shooter 2 " +shooterMotor2.getVelocity());
        } 
        
      } else {
        shooterMotor2.set(0);
        shooterMotor1.set(0);
      }


        if(idleShooting & intakeShooter == false){
          runShooter = true;
        }
        else if(intakeShooter){
          runShooter = true;
        }
        else if (idleShooting == false){
          runShooter = false;
        }
        System.out.println("idle shooting " + idleShooting);
    }
    public void autoSpeed(){
        setRPM = (drive.getPose().getTranslation().getDistance(new Translation2d(16.579342, 5.547))- 1.27) * (6800 - 3500) / (5.7912 -1.27) + 3500;
    } 

    public void IdleRevUp(){
        setRPM = 2000;
        runShooter = true;
      }
    
  
    
    public Command intakeIndexerForShooting(double speed, double s){
      return this.runOnce(()-> shooterIndexerMotor.set(speed)).andThen( new WaitCommand(s)).andThen( setIndexerSpeed(0));
    }

    public Command increaseShooterSpeed() {
        return this.runOnce(() -> shooterSpeedOffset += 50).andThen(()-> System.out.println(shooterSpeedOffset));
    }

    public Command decreaseShooterSpeed() {
        return this.runOnce(() -> shooterSpeedOffset -= 50).andThen(()-> System.out.println(shooterSpeedOffset));
    }

    public Command setRunShooter(boolean runShooter) {
        return this.runOnce(() -> this.runShooter = runShooter);
    }

    public Command setRunIndexer(boolean runIndexer) {
        return this.runOnce(() -> this.runIndexer = runIndexer);
    }

    public Command setAmpandClimbMode(boolean ampAndClimbMode) {
        return this.runOnce(() -> this.ampAndClimbMode = ampAndClimbMode);
    }


    public Command runAutonShooting(boolean side) {
        return new SequentialCommandGroup(setRunShooter(true), setRPMShooter(4000), new WaitCommand(1), setIndexerSpeed(.2),
                new WaitCommand(0.3), setRunShooter(false), setIndexerSpeed(0));
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

    public Command setRPMShooter(double sp) {
        return this.runOnce(() -> this.setRPM = sp);
    }

    /**
     * Sets the Indexer motor speed to a percent between -1 and 1
     *
     * @param
     */
    public Command setIndexerSpeed(double percent) {
        return this.runOnce(() -> shooterIndexerMotor.set(percent));
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
    private void runShooting(double v){
        shooterMotor1.setVelocity(v);
        shooterMotor2.setVelocity(v);
    }

    /**
     * Stops the Shooter Motor
     */

  /**
   * Checks for current spike inside of the indexer
   * @return True if a game piece is in the Indexer
   */
  private boolean currentCheck(){
    new WaitCommand(0.4);
    if (shooterIndexerMotor.getCurrent() < 39.8){
      return false;
    } 
    if (shooterIndexerMotor.getCurrent() > 39.8){
      return true;
    } 
    return false;    
  }

  public boolean reachedSpeed(){
    if(setRPM > 0){
      if (shooterMotor1.getVelocity() >= setRPM -1 && shooterMotor2.getVelocity() >= setRPM-1){
        return true;
      }
    }
    return false;
  }

  /**
   * decides if a game piece is truly inside of 
   * @return true if game piece false if not
   */
  public boolean checkForGamePiece(){
    int piece = 0;
    int noPiece = 0;
    if(intakeShooter){
      if(shooterIndexerMotor.getCurrent() > 39.8){
        for(int i = 0; i <=10; i++){
          if(currentCheck() == true){
            piece++;
          }
        }
      }
    }
        new WaitCommand(0.5);
        if(piece > noPiece){
          idleShooting = true;
          return true;
      }
      return false;
    }
  }


    


   