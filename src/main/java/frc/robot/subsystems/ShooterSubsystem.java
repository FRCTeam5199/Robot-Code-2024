// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    public SparkPIDController shooterMotor1PidController;

    public double shooterSpeed;

    public double shooterSpeedOffset;

    public double setRPM = 0;
    public double speedPercent = 0;
    public double autoAimRPM = 0;

    public PIDController topWheelPIDController;
    public PIDController bottomWheelPIDController;


    public boolean autoTargeting = false;
    public boolean ampMode = false;
    public boolean runShooter = false;
    public boolean runIndexer = false;
    public boolean intakeShooter = false;
    public boolean autonSide = false;
    public boolean idleShooting = false;

    public TalonFX topShooter;
    public TalonFX bottomShooter;

    public VelocityVoltage velocity_request;


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
        velocity_request = new VelocityVoltage(0).withSlot(0).withFeedForward(1).withEnableFOC(true);
    
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


        topShooter = new TalonFX(14);
        bottomShooter = new TalonFX(15);


        SlotConfigs SlotConfigTopShooter = new SlotConfigs();
        SlotConfigs SlotConfigBottomShooter = new SlotConfigs();

        configureSlot(SlotConfigTopShooter, 2.2, 0.1, 0.7, 0.2, 0);
        configureSlot(SlotConfigBottomShooter, 2.2, 0.1, 0.7, 0.2, 0);

        
        topShooter.getConfigurator().apply(SlotConfigTopShooter);
        bottomShooter.getConfigurator().apply(SlotConfigBottomShooter);
        
    }

    @Override
    public void periodic() {
      
        // if(topShooter.getVelocity().getValueAsDouble() > 1){
        //     // System.out.println("top shooter " + topShooter.getVelocity().getValueAsDouble());
        //     // System.out.println("botttom shooter " + bottomShooter.getVelocity().getValueAsDouble());
        //     System.out.println("thingy " + autoAimRPM);
        //     System.out.println("weird thingy + 1" +autoAimRPM/90);
        // }
        if(DriverStation.getAlliance().isPresent()){
            if (DriverStation.getAlliance().get() == Alliance.Red){
                autoAimRPM = ((drive.getPose().getTranslation().getDistance(new Translation2d(16.579342, 5.547)) - 1.27) * (6800 - 3400) / (5.7912 - 1.27) + 3400);


            }
            else if (DriverStation.getAlliance().get() == Alliance.Blue){
                autoAimRPM = ((drive.getPose().getTranslation().getDistance(new Translation2d(0.1, 5.547)) - 1.39) * (6800 - 3400) / (5.7912 - 1.39) + 3400);
                
            }

        // System.out.println(autoAimRPM);
        // System.out.println("super" + autoAimRPM/90);
        }
    }
    

    public void configureSlot(SlotConfigs SlotConfig, double kA, double kV, double kP, double kI, double kD){
        SlotConfig.kA = kA;
        SlotConfig.kV = kV;
        SlotConfig.kP = kP;
        SlotConfig.kI = kI;
        SlotConfig.kD = kD;
    }


    public void autoSpeed() {
       
    }

    
    public Command  autoAim(){
        return this.runOnce(() -> topShooter.setControl(velocity_request.withVelocity(((autoAimRPM/90)+20)))).andThen(() -> bottomShooter.setControl(velocity_request.withVelocity((autoAimRPM/90)))).alongWith(new InstantCommand(()-> System.out.println("value" + autoAimRPM/90 + "  velocity " + autoAimRPM)));
    }
    

    public Command runShooterAtRpm(double vel) {
        return this.runOnce(() -> topShooter.setControl(velocity_request.withVelocity((vel/90)))).andThen(() -> bottomShooter.setControl(velocity_request.withVelocity((vel/90))));/*.alongWith(new InstantCommand(()-> System.out.println("value" + vel/90 + "  velocity " + vel)*/
    }

    public Command runShooterAtPercent(double per) {
        return this.runOnce(() -> topShooter.set(per)).andThen(() -> bottomShooter.set(per));
    }

    public Command runShooterPredeterminedRPM() {
        return this.runOnce(() -> topShooter.setControl(velocity_request.withVelocity((setRPM/90)+ shooterSpeedOffset))).andThen(() -> bottomShooter.setControl(velocity_request.withVelocity((setRPM/90)+ shooterSpeedOffset)));
        }

    public Command runShooterClimbAmp(double vel) {
        return this.runOnce(() -> topShooter.setControl(velocity_request.withVelocity((vel/90)))).andThen(() -> bottomShooter.set(0));
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




    public boolean reachedSpeed() {
        if (setRPM > 0) {
            if (topShooter.getVelocity().getValueAsDouble() >= setRPM - 1 && bottomShooter.getVelocity().getValueAsDouble() >= setRPM - 1) {
                return true;
            }
        }
        return false;
    }
}