package frc.robot.subsystems;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;


public class ArmSubsystem extends SubsystemBase {  
    
    public CANSparkBase armMotorL;
    public CANSparkBase armMotorR;
    public RelativeEncoder encoder;
    public SparkPIDController sparkPIDController;



    private static ArmSubsystem armSubsystem;
    private static AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
    public ArmFeedforward feedforward;

    private static boolean subsystemStatus = false;

    private static boolean isBrakeMode = false;
    public boolean inAuton = false;
    public boolean climbMode = false;
    public double encoderValue;
    public boolean isAiming = true;
    public boolean autoAiming = false;
    public SwerveDrive drive = TunerConstants.DriveTrain;
    private SparkAbsoluteEncoder armEncoder;
    private PIDController rotatePIDController;
    private double rotateSetpoint = 120;
    private double rotateOffset;
    private double pidPercent;

    private ConditionalCommand autoAimArmSide;

    public ArmSubsystem() {
    }

    /**
     * Gets the instnace of the Arm Subsystem.
     */
    public static ArmSubsystem getInstance() {
        if (armSubsystem == null) {
            armSubsystem = new ArmSubsystem();
        }

        return armSubsystem;
    }

    public static void toggleBrakeMode() {
        isBrakeMode = !isBrakeMode;
        // if (isBrakeMode) {
        //     armMotorL.setIdleMode(IdleMode.kBrake);
        //     armMotorR.setIdleMode(IdleMode.kBrake);
        // } else {
        //     armMotorL.setIdleMode(IdleMode.kCoast);
        //     armMotorR.setIdleMode(IdleMode.kCoast);
        // }
    }

    public void setBrakeTrue() {
        armMotorL.setIdleMode(IdleMode.kBrake);
        armMotorR.setIdleMode(IdleMode.kBrake);
    }

    /**
     * init for arm and pid controller
     */
    public void init() {
        motorInit();


        try {
            PIDInit();
        } catch (Exception exception) {
            System.err.println("One or more issues occured while trying to initalize PID for Arm Subsystem");
            System.err.println("Exception Message:" + exception.getMessage());
            System.err.println("Exception Cause:" + exception.getCause());
            System.err.println("Exception Stack Trace:" + exception.getStackTrace());
        }
    }

    public boolean getSubsystemStatus() {
        return subsystemStatus;
    }

    public void motorInit() { 
        armMotorL = new CANSparkFlex(MainConstants.IDs.Motors.ARM_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        armMotorR = new CANSparkFlex(MainConstants.IDs.Motors.ARM_MOTOR2_ID, CANSparkLowLevel.MotorType.kBrushless);

        armMotorL.setInverted(false);
        armMotorL.setIdleMode(IdleMode.kBrake);

        
        armMotorR.setInverted(true);
        armMotorR.setIdleMode(IdleMode.kBrake);

        armMotorL.setSmartCurrentLimit(40);
        armMotorR.setSmartCurrentLimit(40);


        armEncoder = armMotorL.getAbsoluteEncoder(Type.kDutyCycle);


        armMotorL.burnFlash();
        armMotorR.burnFlash();

        
        feedforward = new ArmFeedforward(1, 0.58, 1.35);
    }

    public void PIDInit() {

        rotatePIDController = new PIDController(0.0085262, 0.00569673212, 0.003);
        rotatePIDController.setIZone(3);

    }

    public boolean checkMotors() {
        if (armMotorL != null && armEncoder != null) {
            return true;
        } else {
            return false;
        }
    }

    public boolean checkPID() {
        if (rotatePIDController != null) {
            return true;
        } else {
            return false;
        }

    }

    @Override
    public void periodic() {
         if (armEncoder.getPosition() < 150 && armEncoder.getPosition() > 0) {
            encoderValue = armEncoder.getPosition();
        } else if (armEncoder.getPosition() > 150 && armEncoder.getPosition() < 200) {
            encoderValue = 140;
        } else if (armEncoder.getPosition() > 200 && armEncoder.getPosition() < 361) {
            encoderValue = 0;
        }

        ArmFeedforward feedforward = new ArmFeedforward(1, 0.58, 1.35);

        // double f = rotatePIDController.calculate(encoderValue, 120) + feedforward.calculate(Math.toRadians(120-23), 0);
        

        // System.out.println(f);
        // armMotorL.set(f);
       

        if (checkMotors() && checkPID()) {
            subsystemStatus = true;
        } else {
            subsystemStatus = false;
        }

        if (subsystemStatus) {
            subsystemPeriodic();
        }
    }

    private void subsystemPeriodic() {
       
        if (inAuton) {
            goToSetpoint(rotateSetpoint, rotateOffset);
        }
        if (!autoAiming) {
            if (isAiming) {
                rotatePIDController.setPID(0.0075262, 0.00472673212, 0.00);
                rotatePIDController.setIZone(3);

                goToSetpoint(rotateSetpoint, rotateOffset);
                   System.out.println("encoder " + encoderValue + " desired "  + rotateSetpoint);

            } else {
                goToSetpoint(MainConstants.Setpoints.ARM_STABLE_SETPOINT, rotateOffset);
            }
            
        } else {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                rotateSetpoint = armSpeakersAligningRed();
            } else {
                rotateSetpoint = armSpeakersAligningBlue();
            }
            // if (rotatePIDController.calculate(encoderValue, rotateSetpoint) > 0) {
            //     rotatePIDController.setPID(0.0091262, 0.009273212, 0.00);
            //     rotatePIDController.setIZone(3.4);
            // } else if (rotatePIDController.calculate(encoderValue, rotateSetpoint) < 0) {
            //     rotatePIDController = new PIDController(0.00019262, 0.000409673212, 0.00);
            //     rotatePIDController.setIZone(3);
            // }
            goToSetpoint(rotateSetpoint, 0);

            
            System.out.println("encoder " + encoderValue + "desired "  + rotateSetpoint);
        }
    }

    private void goToSetpoint(double rotateSetpoint, double rotateOffset) {
        // System.out.println("Rotate Setpoint: " + rotateSetpoint);
        // System.out.println("Arm Position: " + armEncoder.getPosition());

        if (encoderValue < 0 || rotateSetpoint < 0) return;
        if (climbMode) {
            // armMotorL.set(rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset) * 0.5);
            // armMotorR.set(rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset) * 0.5);
            armMotorL.set(rotatePIDController.calculate(encoderValue, rotateSetpoint) + feedforward.calculate(Math.toRadians(rotateSetpoint-23), 0));
        } else {
            armMotorL.set(rotatePIDController.calculate(encoderValue, rotateSetpoint) + feedforward.calculate(Math.toRadians(rotateSetpoint-23), 0));
        }
    }

    public double armSpeakersAligningRed() {
        double angleForArm;
        double distanceFromRobot;
        distanceFromRobot = drive.getPose().getTranslation().getDistance(new Translation2d(16.479342, 5.547));

        // double distanceAimSpeaker = (drive.getPose().getTranslation().getDistance(new Translation2d(16.579342, 5.547)) - 1.27) * (0.700 - 0.645) / (5.7912 - 1.27) + 0.645;
        double speakerHeight = MainConstants.SPEAKER_Z - MainConstants.ARM_PIVOT_Z;
        distanceFromRobot += MainConstants.ARM_PIVOT_X_OFFSET;
        angleForArm = Math.toDegrees(Math.atan(speakerHeight / distanceFromRobot)) + MainConstants.ARM_ORIGINAL_DEGREES;

        return angleForArm;
    }

    public double armSpeakersAligningBlue() {
        double angleForArm;
        double distanceFromRobot;
        distanceFromRobot = drive.getPose().getTranslation().getDistance(new Translation2d(0.1, 5.547));

        // double distanceAimSpeaker = (drive.getPose().getTranslation().getDistance(new Translation2d(0.01, 5.547)) - 1.27) * (0.700 - 0.645) / (5.7912 - 1.27) + 0.645;
        double speakerHeight = (MainConstants.SPEAKER_Z) - MainConstants.ARM_PIVOT_Z;
        distanceFromRobot += MainConstants.ARM_PIVOT_X_OFFSET;
        angleForArm = Math.toDegrees(Math.atan(speakerHeight / distanceFromRobot)) + MainConstants.ARM_ORIGINAL_DEGREES;

        return angleForArm;
    }


    public Command switchAiming() {
        return this.runOnce(() -> autoAiming = !autoAiming).andThen(() -> System.out.println(autoAiming));
    }

    public Command isAutoAiming(boolean bool) {
        return this.runOnce(() -> autoAiming = bool);
    }

    public Command setClimbMode(boolean bool) {
        return this.runOnce(() -> climbMode = bool);
    }


    public Command isAiming(boolean bool) {
        return this.runOnce(() -> isAiming = bool);
    }

    public Command increaseOffset(double amount) {
        return this.runOnce(() -> rotateOffset += amount);
    }

    public Command decreaseOffset(double amount) {
        return this.runOnce(() -> rotateOffset -= amount);
    }

    public AbsoluteEncoder getArmEncoder() {
        if (!subsystemStatus) return null;
        return armEncoder;
    }

    /**
     * @param percent move armMotor at a percent(-1 to 1)
     * @return command to spin motor at percent
     */
    public Command moveAtPercent(double percent) {
        return this.run(() -> armMotorL.set(percent));
    }


    /**
     * @param setpoint set armMotor to setPoint
     * @return command to move armMotor to setPoint
     */
    public Command setArmSetpoint(double setpoint) {
        return this.runOnce(() -> System.out.println("changed to " + setpoint)).andThen(() -> rotateSetpoint = setpoint);
    }


    /**
     * Sets the Arm setpoint to the Arm Stable setpoint
     */
    public Command rotateStable() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_STABLE_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Amp setpoint
     */
    public Command rotateAmp() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_AMP_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Back setpoint
     */
    public Command rotateBack() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_SPEAKER_BACK_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Front setpoint
     */
    public Command rotateFront() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_SPEAKER_FRONT_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to slightly above retracted intake
     */
    public Command rotateIntake() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_INTAKE_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm climb setpoint
     */
    public Command rotateTrap() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TRAP_SETPOINT);
    }

    public Command rotatePrepClimbP2() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TRAP_PREP2_SETPOINT);
    }

    public Command rotateTopPiece() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TOP_PIECE_SETPOINT);
    }

    public Command rotateBottomPiece() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_BOTTOM_PIECE_SETPOINT);
    }

    public Command rotateMiddlePiece() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_MIDDLE_PIECE_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Trap setpoint
     */
    public Command rotateTrapPrep() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TRAP_PREP_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Subwoofer setpoint
     */
    public Command rotateSub() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_SUBWOOFER_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Podium setpoint
     */
    public Command rotateSafe() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_SAFE_SETPOINT);
    }

    public Command rotateFarShot() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_FAR_SHOT_SETPOINT);
    }

    public Command rotateHPStation() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_HP_STATION_SETPOINT);
    }
}