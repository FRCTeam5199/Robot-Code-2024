// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new indexer. */
  public VortexMotorController shooterIndexerMotor;

  public IndexerSubsystem() {

  }

  public void init(){
    shooterIndexerMotor = new VortexMotorController(MainConstants.IDs.Motors.SHOOTER_INDEXER_MOTOR_ID);

      
      shooterIndexerMotor.setInvert(false);
      shooterIndexerMotor.setBrake(true);

      shooterIndexerMotor.setCurrentLimit(40);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command intakeIndexerForShooting(double speed, double s) {
        return this.runOnce(() -> shooterIndexerMotor.set(speed)).andThen(new WaitCommand(s)).andThen(setIndexerSpeed(0));
      }
      public Command setIndexerSpeed(double percent) {
        return this.runOnce(() -> shooterIndexerMotor.set(percent));
    }
        /**
     * Stops the Shooter Motor
     */

    /**
     * Checks for current spike inside of the indexer
     *
     * @return True if a game piece is in the Indexer
     */
    private boolean currentCheck() {
      new WaitCommand(0.4);
      if (shooterIndexerMotor.getCurrent() < 39.8) {
          return false;
      }
      if (shooterIndexerMotor.getCurrent() > 39.8) {
          return true;
      }
      return false;
  }
  public boolean checkForGamePiece() {
    int piece = 0;
    int noPiece = 0;
        if (shooterIndexerMotor.getCurrent() > 39.8) {
            for (int i = 0; i <= 10; i++) {
                if (currentCheck() == true) {
                    piece++;
                }
            }
        }
    new WaitCommand(0.5);
    if (piece > noPiece) {
        return true;
    }
    return false;
}
}
