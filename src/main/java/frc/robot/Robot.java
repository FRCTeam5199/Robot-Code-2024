// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Power;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.CANBus;
import com.revrobotics.SparkFlexExternalEncoder;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.UserInterface;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.Optional;

import javax.swing.plaf.basic.BasicBorders.RadioButtonBorder;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot{
    public static final boolean SECOND_TRY = false;

    Pose3d poseA = new Pose3d();
    Pose3d poseB = new Pose3d();

    SwerveDrive drive = TunerConstants.DriveTrain;
    UserInterface userInterface = new UserInterface();
    AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;


    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.

        userInterface.initalizeConfigTab();
        userInterface.initalizeTestTab();
        // userInterface.initalizeGameTab();

        m_robotContainer = new RobotContainer();
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        Logger.start();
    }

    /**
     * This function is called ev  ery 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        
        Logger.recordOutput("Drive/Pose", drive.getPose());


        Logger.recordOutput("MyPose3d", poseA);
        Logger.recordOutput("MyPose3dArray", poseA, poseB);
        Logger.recordOutput("MyPose3dArray", new Pose3d[] { poseA, poseB });

        Logger.recordOutput("sticky fault boot during enbale ", drive.getPigeon2().getFault_BootDuringEnable().getValueAsDouble());
        Logger.recordOutput("boot during enbale ", drive.getPigeon2().getStickyFault_BootDuringEnable().getValueAsDouble());
        

        Logger.recordOutput("sticky fault boot during enbale boolean ", drive.getPigeon2().getFault_BootDuringEnable().getValue());
        Logger.recordOutput("sticky fault boot during enbale boolean", drive.getPigeon2().getFault_BootDuringEnable().getValue());
        
        Logger.recordOutput("heading of pigeon", drive.getPigeon2().getAngle());
        Logger.recordOutput("supply voltage voltage of pigeon", drive.getPigeon2().getSupplyVoltage().getValueAsDouble());

        PowerDistribution power = new PowerDistribution(62, ModuleType.kRev);
        Logger.recordOutput("get Voltage of battery", power.getVoltage());
        power.close();
        try{
        Logger.recordOutput("front camera alive", aprilTagSubsystem.frontCamera.isConnected());
        Logger.recordOutput("back camera alive", aprilTagSubsystem.backCamera.isConnected());
        } catch (Exception e){
            
        }
        // Logger.recordOutput("front camera alive", null);
        // Logger.recordOutput("front camera alive", null);

        CommandScheduler.getInstance().run();
        // System.out.println(drive.getPose());
        Optional<EstimatedRobotPose> estimatePose1 = aprilTagSubsystem.getVisionPoseFront();
        //Optional<EstimatedRobotPose> estimatePose2 = aprilTagSubsystem.getVisionPoseRight();
        // Optional<EstimatedRobotPose> estimatePose3 = aprilTagSubsystem.getVisionPoseLeft();
        // Optional<EstimatedRobotPose> estimatePose4 = aprilTagSubsystem.getVisionPoseBack();
   if(estimatePose1.isPresent()){
      EstimatedRobotPose robotPose = estimatePose1.get();
      drive.addVisionMeasurement(robotPose.estimatedPose.toPose2d(), Timer.getFPGATimestamp());
   }

    
    // if(estimatePose2.isPresent()){
    //     EstimatedRobotPose robotPose = estimatePose2.get();
    //     drive.addVisionMeasurement(robotPose.estimatedPose.toPose2d(), Timer.getFPGATimestamp());
    // }
        // if(estimatePose3.isPresent()){
        //   EstimatedRobotPose robotPose = estimatePose3.get();
        //   drive.addVisionMeasurement(robotPose.estimatedPose.toPose2d(), Timer.getFPGATimestamp());
        // }
        // if(estimatePose4.isPresent()){
        //   EstimatedRobotPose robotPose = estimatePose4.get();
        //   drive.addVisionMeasurement(robotPose.estimatedPose.toPose2d(), Timer.getFPGATimestamp());
        // }

        // userInterface.updateGameTab();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        m_robotContainer.arm.setBrakeTrue();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.arm.setBrakeTrue();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
