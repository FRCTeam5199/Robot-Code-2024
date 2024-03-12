package frc.robot.subsystems;

import java.util.zip.ZipEntry;

import org.json.simple.parser.Yytoken;

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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
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
    private PIDController voltagePIDController;
    private double rotateSetpoint = 120;

    public final double horizontalOffset = 21.7;

    private double rotateOffset;
    private double pidPercent;

    public ArmFeedforward feedfoward;
    public TrapezoidProfile trapProfile;
    // degrees/sec
    public final TrapezoidProfile.Constraints trapConstraints =  new TrapezoidProfile.Constraints(45, 90);
    public final Timer timer = new Timer();

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

        setTrapezoidalProfileSetpoint(120);
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
        trapProfile = new TrapezoidProfile(trapConstraints, new TrapezoidProfile.State(120,0), new TrapezoidProfile.State(encoderValue, armEncoder.getVelocity()*360.0));
     
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

        
    }

    public void PIDInit() {

    rotatePIDController = new PIDController(0.06, 0.00569673212, 0.00);
    voltagePIDController = new PIDController(0.0, 0, 0);
    rotatePIDController.setIZone(3);


        // KS units = volts to overcome static friction
        // KG units = volts to compensate for gravity when the arm is horizontal
        // KV units = volts / (radians per second)
        // feedforward = new ArmFeedforward(0.077, 0.253, 6); // requires radians
        feedforward = new ArmFeedforward(0.077, 0.253, 1.42);

        setTrapezoidalProfileSetpoint(120);
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
         if (armEncoder.getPosition() < 200 && armEncoder.getPosition() > 0) {
            encoderValue = armEncoder.getPosition();
        } else if (armEncoder.getPosition() > 200 && armEncoder.getPosition() < 250) {
            encoderValue = 140;
        } else if (armEncoder.getPosition() > 250 && armEncoder.getPosition() < 361) {
            encoderValue = 0;
        }
        if(DriverStation.isEnabled()){
            // System.out.println("encoder " + encoderValue + "value " + rotateSetpoint);
        }
        // System.out.println("encoder value" + encoderValue);
        System.out.println("velocity " + armEncoder.getVelocity());
        
        goToSetpoint(rotateOffset);
        // armMotorL.setVoltage(0.9+(0.077+ 0.253));

        // armProfile.calculate(1, new TrapezoidProfile.State(encoderValue, 0), new TrapezoidProfile.State(120, 0));
                // armMotorL.set
        // PID proportional integral derivative
        // kp --> what's the unit --> x per y
        // x the motion, y is the unit error
        // kp --> voltage per unit of error
        // PID(5, 2, 1) volts per degree of error --> new PID(5 * 360,  2 * 360, 1 * 360) with units ovlts per rotation of error

        // armMotorL.setVoltage(ff voltage + pid voltage);


        // PositionVoltage -- combined position request w/ the ability to augment it with voltage feed forward

        
        // armMotorL.setVoltage(
        //     pidController.calculate(encoderValue) a
        //         + feedforward.calculate(Math.toRadians(pidController.getSetpoint().position+23), pidController.getSetpoint().velocity));
        

        // armMotorL.setVoltage(0.33); //0.176 // 0.33 / 21.7 offset Kg = 0.253
        // max = kg + ks
        // min = kg - ks
        // max + min = 2kg
        // kg = 0.253
        // 0.33 = 0.253 + ks
        // ks = 0.077


                //  Math.toRadians((pidController.getSetpoint().position+23))
        // double f = rotatePIDController.calculate(encoderValue, 120) + feedforward.calculate(Math.toRadians(120-23), 0);
        

        // System.out.println(f);
        // armMotorL.set(f);
       

        if (checkMotors() && checkPID()) {
            subsystemStatus = true;
        } else {
            subsystemStatus = false;
        }

        if (subsystemStatus) {
            if (DriverStation.isEnabled()){
            // subsystemPeriodic( );
            }
        }
    }

    private void subsystemPeriodic() {
        
        if(isAiming){
          
            goToSetpoint(rotateOffset);
        }
        else{
            if(rotatePIDController.calculate(encoderValue, 23) > 0){
                rotatePIDController.setPID(0.0085262,0.00569673212 , 0);
            }
            else{
                rotatePIDController.setPID(0.0045262,0.00569673212 , 0);
            }

            // armMotorL.set(rotatePIDController.calculate(encoderValue, MainConstants.Setpoints.ARM_STABLE_SETPOINT));
            armMotorL.set(rotatePIDController.calculate(encoderValue, MainConstants.Setpoints.ARM_STABLE_SETPOINT));
        }
        // if (inAuton) {
        //     goToSetpoint(rotateOffset);
        // }
        // if (!autoAiming) {
        //     if (isAiming) {
        //         goToSetpoint(rotateOffset);
        //     } else {
        //         goToSetpoint(rotateOffset);
        //     }
        //     System.out.println("encoder " + encoderValue + "desired "  + rotateSetpoint);
            
        // } else {
        //     if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        //         setTrapezoidalProfileSetpoint( armSpeakersAligningRed());
        //     } else {
        //         setTrapezoidalProfileSetpoint( armSpeakersAligningBlue());
        //     }
        //     System.out.println("encoder " + encoderValue + "desired "  + rotateSetpoint);

        // }   
    }


    public void goToSetpoint(double rotateOffset) {

        if (encoderValue < 0 || rotateSetpoint < 0) return;

        TrapezoidProfile.State goalState = trapProfile.calculate(timer.get());

        // System.out.println("goal " + goalState.velocity);
        // System.out.println("position" + goalState.position);
        if (climbMode) {
            armMotorL.setVoltage(voltagePIDController.calculate(encoderValue, goalState.position) + feedforward.calculate(Math.toRadians((encoderValue-horizontalOffset)), Math.toRadians(goalState.velocity)));
        } else {
            armMotorL.setVoltage(voltagePIDController.calculate(encoderValue, goalState.position) + feedforward.calculate(Math.toRadians((encoderValue-horizontalOffset)), Math.toRadians(goalState.velocity)));
            
        }
        // System.out.println(voltagePIDController.calculate(encoderValue, goalState.position) + feedforward.calculate(Math.toRadians((encoderValue-horizontalOffset)), goalState.velocity));

        //rotateSetpoint - encoderValue = direction
        // current position and target position
        // feedforward.calculate(current arm angle, velocity)
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
        return this.runOnce(() -> System.out.println("changed to " + setpoint)).andThen(() -> setTrapezoidalProfileSetpoint(setpoint));
    }


    /**
     * Sets the Arm setpoint to the Arm Stable setpoint
     */
    public Command rotateStable() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_STABLE_SETPOINT));
    }

    /**
     * Sets the Arm setpoint to the Arm Amp setpoint
     */
    public Command rotateAmp() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_AMP_SETPOINT));
    }

    /**
     * Sets the Arm setpoint to the Arm Back setpoint
     */
    public Command rotateBack() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_SPEAKER_BACK_SETPOINT));
    }

    /**
     * Sets the Arm setpoint to the Arm Front setpoint
     */
    public Command rotateFront() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_SPEAKER_FRONT_SETPOINT));
    }

    /**
     * Sets the Arm setpoint to slightly above retracted intake
     */
    public Command rotateIntake() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_INTAKE_SETPOINT));
    }

    /**
     * Sets the Arm setpoint to the Arm climb setpoint
     */
    public Command rotateTrap() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_TRAP_SETPOINT));
    }

    public Command rotatePrepClimbP2() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_TRAP_PREP2_SETPOINT));
    }

    public Command rotateTopPiece() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_TOP_PIECE_SETPOINT));
    }

    public Command rotateBottomPiece() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_BOTTOM_PIECE_SETPOINT));
    }

    public Command rotateMiddlePiece() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_MIDDLE_PIECE_SETPOINT));
    }

    /**
     * Sets the Arm setpoint to the Arm Trap setpoint
     */
    public Command rotateTrapPrep() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_TRAP_PREP_SETPOINT));
    }

    /**
     * Sets the Arm setpoint to the Arm Subwoofer setpoint
     */
    public Command rotateSub() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_SUBWOOFER_SETPOINT));
    }

    /**
     * Sets the Arm setpoint to the Arm Podium setpoint
     */
    public Command rotateSafe() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_SAFE_SETPOINT));
    }

    public Command rotateFarShot() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint( MainConstants.Setpoints.ARM_FAR_SHOT_SETPOINT));
    }

    public Command rotateHPStation() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint(MainConstants.Setpoints.ARM_HP_STATION_SETPOINT));
    }

    private void setTrapezoidalProfileSetpoint(double setpoint) {
        // todo
        
        rotateSetpoint= setpoint;
        // trap constraints degrees
        // rotateSetpoint  degrees?
        //
        // if(climbMode) {
        // trapProfile = new TrapezoidProfile(climbTrapezoidalConstraints, new TrapezoidProfile.State(rotateSetpoint,0), new TrapezoidProfile.State(encoderValue, armEncoder.getVelocity()*360.0));

        // } else {
        trapProfile = new TrapezoidProfile(trapConstraints, new TrapezoidProfile.State(rotateSetpoint,0), new TrapezoidProfile.State(encoderValue, armEncoder.getVelocity()*360.0));
        // }
        timer.restart();        
    }
}