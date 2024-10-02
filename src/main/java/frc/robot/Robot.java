// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LED.LEDSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.UserInterface;
import frc.robot.subsystems.drivetrain.SwerveDrive;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
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

        m_robotContainer = new RobotContainer();

        userInterface.initalizeConfigTab();
        userInterface.initalizeTestTab();
        // userInterface.initalizeGameTab();
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

        m_robotContainer.periodic();
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.

        Logger.recordOutput("Drive/Pose", drive.getPose());


        Logger.recordOutput("MyPose3d", poseA);
        Logger.recordOutput("MyPose3dArray", poseA, poseB);
        Logger.recordOutput("MyPose3dArray", new Pose3d[]{poseA, poseB});

        Logger.recordOutput("sticky fault boot during enbale ", drive.getPigeon2().getFault_BootDuringEnable().getValueAsDouble());
        Logger.recordOutput("boot during enbale ", drive.getPigeon2().getStickyFault_BootDuringEnable().getValueAsDouble());


        Logger.recordOutput("sticky fault boot during enbale boolean ", drive.getPigeon2().getFault_BootDuringEnable().getValue());
        Logger.recordOutput("sticky fault boot during enbale boolean", drive.getPigeon2().getFault_BootDuringEnable().getValue());

        Logger.recordOutput("heading of pigeon", drive.getPigeon2().getAngle());
        Logger.recordOutput("supply voltage voltage of pigeon", drive.getPigeon2().getSupplyVoltage().getValueAsDouble());

        PowerDistribution power = new PowerDistribution(62, ModuleType.kRev);
        Logger.recordOutput("get Voltage of battery", power.getVoltage());
        power.close();

        Logger.recordOutput("top shooter current supply", m_robotContainer.shooterSubsystem.topShooter.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("bottom shooter current supply", m_robotContainer.shooterSubsystem.bottomShooter.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput("top shooter supply voltage", m_robotContainer.shooterSubsystem.topShooter.getSupplyVoltage().getValueAsDouble());
        Logger.recordOutput("bottom shooter supply voltage", m_robotContainer.shooterSubsystem.bottomShooter.getSupplyVoltage().getValueAsDouble());

        // Logger.recordOutput("front camera alive", null);
        // Logger.recordOutput("front camera alive", null);

        CommandScheduler.getInstance().run();
        // System.out.println(drive.getPose());
        Optional<EstimatedRobotPose> estimatePose1 = aprilTagSubsystem.getEstimatedGlobalPoseFront();
        // Optional<EstimatedRobotPose> estimatePose2 = aprilTagSubsystem.getVisionPoseRight();
        // Optional<EstimatedRobotPose> estimatePose3 = aprilTagSubsystem.getVisionPoseLeft();
        Optional<EstimatedRobotPose> estimatePose4 = aprilTagSubsystem.getEstimatedGlobalPoseBack();

        if (!DriverStation.isAutonomous()) {
            if (estimatePose4.isPresent()) {

                EstimatedRobotPose robotPose = estimatePose4.get();

                Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();

                Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(), robotPose2d.getRotation());
//                Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0)

                TunerConstants.DriveTrain.addVisionMeasurement(modify, aprilTagSubsystem.getTimestamp());
            }

            if (estimatePose1.isPresent()) {

                EstimatedRobotPose robotPose = estimatePose1.get();

                Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();

                Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(), robotPose2d.getRotation());
                //Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 0 : 180)

                TunerConstants.DriveTrain.addVisionMeasurement(modify, aprilTagSubsystem.getTimestamp());
            }
        }

        // userInterface.updateGameTab();

    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */

    @Override
    public void disabledInit() {
        m_robotContainer.onDisable();
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.disabledPeriodic();
    }

    @Override
    public void disabledExit() {
        m_robotContainer.onEnable();
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
        // m_robotContainer.arm.setBrakeTrue();

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
        // m_robotContainer.arm.setBrakeTrue();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        // System.out.println(new Pose2d(16.58, 5.54, new Rotation2d(0)).getTranslation().getDistance(drive.getPose().getTranslation()));
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
