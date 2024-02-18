// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem shooterSubsystem;

    public VortexMotorController shooterMotor1;
    public VortexMotorController shooterMotor2;

    public VortexMotorController shooterIndexerMotor;

    public double shooterSpeed;
    public double indexerSpeed;
    public double shooterSpeedOffset;

    public double setSpeed = 0;

    public boolean ampAndClimbMode = false;
    public boolean runShooter = false;
    public boolean runIndexer = false;
    public boolean intakeShooter = false;
    public boolean autonSide = false;

    public GenericHID genericHID = new GenericHID(0);

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

        // Shuffleboard.getTab("Test").add("Shooter Subsystem Initalized", true).getEntry();
    }

    /*
     * Initalizes the motor(s) for this subsystem
     */
    public void motorInit() {
        shooterMotor1 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_1_ID);
        shooterMotor2 = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_MOTOR_2_ID);
        shooterIndexerMotor = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_INDEXER_MOTOR_ID);

        shooterMotor1.setInvert(true);
        shooterMotor2.setInvert(false);

        shooterIndexerMotor.setInvert(false);
        shooterIndexerMotor.setBrake(true);

        shooterMotor1.getEncoder().setPosition(0);
        shooterMotor2.getEncoder().setPosition(0);

        shooterMotor1.setCurrentLimit(40);
        shooterMotor2.setCurrentLimit(40);
        shooterIndexerMotor.setCurrentLimit(40);
    }

    @Override
    public void periodic() {
    
      // System.out.println(checkForGamePiece());
        // This method will be called once per scheduler run

        // if (checkForGamePiece()) {
        //   genericHID.setRumble(RumbleType.kBothRumble, 1);
        // } else {
        //   genericHID.setRumble(RumbleType.kBothRumble, 0);
        // }

        if (ampAndClimbMode) {
            shooterSpeed = 0.2;
            indexerSpeed = 0.5;
        } else if (intakeShooter) {
            shooterSpeed = -0.3;
            indexerSpeed = -0.4;
        } else if (autonSide) {
            shooterSpeed = 0.7;
            indexerSpeed = 0.5;
        } else {
            shooterSpeed = setSpeed;
            indexerSpeed = 0.5;
        }

        if (intakeShooter) {
            shooterMotor1.set(-.3);
            shooterMotor2.set(-.3);
        } else {
            if (runShooter) {
                if (!ampAndClimbMode) {
                    shooterMotor1.set((shooterSpeed + shooterSpeedOffset) -0.05);
                }
                shooterMotor2.set((shooterSpeed + shooterSpeedOffset));
                System.out.println("velocity of bottom " +shooterMotor1.getVelocity());
                System.out.println("velocity of top " +shooterMotor2.getVelocity());
            } else {
                shooterMotor1.set(0);
                shooterMotor2.set(0);
            }
        }
        if (intakeShooter) {
            shooterIndexerMotor.set(-.3);
        } else {

            if (runIndexer) {
                shooterIndexerMotor.set(indexerSpeed);
            } else {
                shooterIndexerMotor.set(0);
            }
        }
    }
//  public Command AutonShooting(double Shooter, double Indexer){
//    return this.runOnce();
//  }

    public Command increaseShooterSpeed() {
        return this.runOnce(() -> shooterSpeedOffset += 0.05);
    }

    public Command decreaseShooterSpeed() {
        return this.runOnce(() -> shooterSpeedOffset -= 0.05);
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
        return new SequentialCommandGroup(autonSpeed(side), setRunShooter(true), new WaitCommand(.5), setRunIndexer(true),
                new WaitCommand(0.3), setRunShooter(false), setRunIndexer(false), autonSpeed(false));
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
    public Command setSpeedOfShooter(double sp){
      return this.runOnce(()-> this.setSpeed = sp);
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
        return this.runOnce(() -> setSpeed = percent);
    }

    /**
     * Sets the Shooter motor velocity based on the RPM of the motor
     *
     * @param
     */
    public Command setShooterVelocity(double velocity) {
        return this.runOnce(() -> shooterMotor1.setVelocity(velocity)).andThen(() -> shooterMotor2.setVelocity(velocity));
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
      if (shooterMotor1.getSpeed()+0.01 > shooterSpeed){
        return true;
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
          return true;
      }
      return false;
    }
  }


    


   