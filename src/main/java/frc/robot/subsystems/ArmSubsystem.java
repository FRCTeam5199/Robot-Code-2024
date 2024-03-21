package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;


public class ArmSubsystem extends SubsystemBase {

    private static ArmSubsystem armSubsystem;
    private static AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
    private static boolean subsystemStatus = false;
    private static boolean isBrakeMode = false;
    public final double horizontalOffset = 29;
    public TalonFX armMotorL;
    public TalonFX armMotorR;
    public CANcoder armCANCoder;
    public MotionMagicVoltage m_MotionMagicVoltageRequest;

    public ArmFeedforward feedforward;


    public boolean inAuton = false;
    public boolean climbMode = false;
    public double encoderValue;
    public boolean isAiming = true;
    public boolean autoAiming = false;
    public SwerveDrive drive = TunerConstants.DriveTrain;

    // private RelativeEncoder armEncoder;


    private PIDController rotatePIDController;
private PIDController voltagePIDController;
    private double rotateSetpoint = 120;

    // setpoint to go to when not in stable position
    public double setPointInQue;


    private double rotateOffset;
    private double pidPercent;

    public ArmFeedforward feedfoward;
    public TrapezoidProfile trapProfile;
    // degrees/sec
    // public final TrapezoidProfile.Constraints trapConstraints =  new TrapezoidProfile.Constraints(270, 727);
    public  TrapezoidProfile.State prevState = new TrapezoidProfile.State(0, 0);
    //undershoot usually maximum mehanical reached
    //overshoot (what kA is todo) 
    public final TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints( 200, 200);
    public final TrapezoidProfile.Constraints crossTrapContrainrs = new TrapezoidProfile.Constraints(200, 200);
    public final TrapezoidProfile.Constraints climbTrapConstraints = new TrapezoidProfile.Constraints(200, 200);
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
        armMotorL.setNeutralMode(NeutralModeValue.Brake);
        armMotorR.setNeutralMode(NeutralModeValue.Brake);

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
    // trapProfile = new TrapezoidProfile(trapConstraints, new TrapezoidProfile.State(120,0), new TrapezoidProfile.State(encoderValue, ((armEncoder./80)*360)));
    setTrapezoidalProfileSetpoint(120);
    }

    public boolean getSubsystemStatus() {
        return subsystemStatus;
    }

    public void motorInit() {
        armMotorL = new TalonFX(MainConstants.IDs.Motors.ARM_MOTOR_LEFT_ID);
        armMotorR = new TalonFX(MainConstants.IDs.Motors.ARM_MOTOR_RIGHT_ID);

        armMotorL.setInverted(false);
        armMotorL.setNeutralMode(NeutralModeValue.Brake);

        armMotorR.setInverted(true);
        armMotorR.setNeutralMode(NeutralModeValue.Brake);
//        armMotorR.setControl(new Follower(armMotorL.getDeviceID(), true));

        
        armCANCoder = new CANcoder(50);


        SlotConfigs SlotConfigLeft = new SlotConfigs();
        SlotConfigs SlotConfigRigth = new SlotConfigs();

        configureSlot(SlotConfigLeft,0,1 ,0, 0, 0, 0, 0);
        configureSlot(SlotConfigRigth,0,1, 0, 0, 0, 0, 0);
        


        armMotorL.getConfigurator().apply(SlotConfigLeft);
        armMotorR.getConfigurator().apply(SlotConfigRigth);
        
        CANcoderConfiguration CANCoder_Config = new CANcoderConfiguration();

        armCANCoder.getConfigurator().apply(CANCoder_Config);

        TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
        fx_cfg.Feedback.FeedbackRemoteSensorID = armCANCoder.getDeviceID();
        fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        fx_cfg.Feedback.SensorToMechanismRatio = 1;
        fx_cfg.Feedback.RotorToSensorRatio = 80;

        armMotorL.getConfigurator().apply(fx_cfg.Feedback);


        
        

        armMotorL.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true));
        armMotorR.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true));



        m_MotionMagicVoltageRequest = new MotionMagicVoltage(120.0/360.0);
        
        
        
    }

    public void PIDInit() {


        rotatePIDController = new PIDController(0.03, 0.00569673212, 0.00);
        voltagePIDController = new PIDController(0.09, 0, 0);

        // KS units = volts to overcome static friction
        // KG units = volts to compensate for gravity when the arm is horizontal
        // KV units = volts / (radians per second)
// feedforward = new ArmFeedforward(0.077, 0.253, 6); // requires radians
        // feedforward = new ArmFeedforward(0.077, 0.253, 1.36);
        feedforward = new ArmFeedforward(0.077, 0.253, 1.35);

    }

    public boolean checkMotors() {
        if (armMotorL != null && armCANCoder != null) {
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
        //        System.out.println(armEncoder.getAbsolutePosition());
        // if (armCANCoder.getAbsolutePosition().getValueAsDouble() < 200 && armCANCoder.getAbsolutePosition().getValueAsDouble() > 0) {
        //     encoderValue = (armCANCoder.getAbsolutePosition().getValueAsDouble());
        // } else if (armCANCoder.getAbsolutePosition().getValueAsDouble() > 200 && armCANCoder.getAbsolutePosition().getValueAsDouble()< 250) {
        //     encoderValue = armCANCoder.getAbsolutePosition().getValueAsDouble();
        // } else if (armCANCoder.getAbsolutePosition().getValueAsDouble() > 250 && armCANCoder.getAbsolutePosition().getValueAsDouble() < 361) {
        //     encoderValue = 0;
        // }
        
        System.out.println(armCANCoder.getAbsolutePosition().getValueAsDouble());
            StrictFollower f = new StrictFollower(30);

        if(DriverStation.isEnabled()){

        // armMotorL.setControl(m_MotionMagicVoltageRequest);
        
        // armMotorR.setControl(f);
        }


        // System.out.println("encoder value" + encoderValue);
        
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
        //     pidController.calculate(encoderValue)
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
            //            subsystemPeriodic();
            }

        
        }
    }


    private void subsystemPeriodic() {

        // if(isAiming){
          
        //     goToSetpoint(rotateOffset);
        // }
        // else{
        //     if(rotatePIDController.calculate(encoderValue, 23) > 0){
        //         rotatePIDController.setPID(0.0085262,0.00569673212 , 0);
        //     }
        //     else{
        //         rotatePIDController.setPID(0.0045262,0.00569673212 , 0);
        //     }

        //     // armMotorL.set(rotatePIDController.calculate(encoderValue, MainConstants.Setpoints.ARM_STABLE_SETPOINT));
        //     armMotorL.set(rotatePIDController.calculate(encoderValue, MainConstants.Setpoints.ARM_STABLE_SETPOINT));
        // }
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

            goToSetpoint(rotateOffset);
                        // System.out.println("setPoint que" + setPointInQue);
            // System.out.println("desired setpoint" + rotateSetpoint);
            // System.out.println("encoder " + encoderValue);
            if (autoAiming){
                if(DriverStation.getAlliance().get() == Alliance.Red){
                    setPointInQue = armSpeakersAligningRed();
                }
                else if(DriverStation.getAlliance().get() == Alliance.Blue){
                    setPointInQue = armSpeakersAligningBlue();
                }
            }
        }
    public void configureSlot(SlotConfigs SlotConfig, double kS,double kG,double kA, double kV, double kP, double kI, double kD){
        SlotConfig.kS = kS;
        SlotConfig.kG = kG;
        SlotConfig.kA = kA;
        SlotConfig.kV = kV;
        SlotConfig.kP = kP;
        SlotConfig.kI = kI;
        SlotConfig.kD = kD;
    }

    
public Command testArm() {
        return this.runOnce(() -> rotateSetpoint = 80);
}
    
public void goToSetpoint(double rotateOffset) {
        
        if (encoderValue < 0 || rotateSetpoint < 0) return;

        armMotorL.set(rotatePIDController.calculate(armMotorL.getPosition().getValue(), rotateSetpoint));
        System.out.println(rotatePIDController.calculate(armMotorL.getPosition().getValue(), rotateSetpoint));
//        TrapezoidProfile.State goalState = trapProfile.calculate(timer.get());

        //        if (climbMode) {
            //            armMotorL.setVoltage(voltagePIDController.calculate(encoderValue, prevState.position) + feedforward.calculate(Math.toRadians((encoderValue-horizontalOffset)), Math.toRadians(goalState.velocity)));
        //        } else {
            //            armMotorL.setVoltage(voltagePIDController.calculate(encoderValue, prevState.position) + feedforward.calculate(Math.toRadians((encoderValue-horizontalOffset)), Math.toRadians(goalState.velocity)));
            //
//        }
//        prevState = goalState;
        // System.out.println(voltagePIDController.calculate(encoderValue, goalState.position) + feedforward.calculate(Math.toRadians((encoderValue-horizontalOffset)), goalState.velocity));

        //rotateSetpoint - encoderValue = direction
        // current position and target position
        // feedforward.calculate(current arm angle, velocity)
        }
    


    public double  armSpeakersAligningRed() {
        double angleForArm;
        double distanceFromRobot;
        distanceFromRobot = drive.getPose().getTranslation().getDistance(new Translation2d((16.479342), 5.547));
        // System.out.println("distance from robot ///////////////////////////////////////" + distanceFromRobot);

        // double distanceAimSpeaker = (drive.getPose().getTranslation().getDistance(new Translation2d(16.579342, 5.547)) - 1.27) * (0.700 - 0.645) / (5.7912 - 1.27) + 0.645;
        double speakerHeight = (MainConstants.SPEAKER_Z - MainConstants.ARM_PIVOT_Z)-0.015;
        distanceFromRobot += MainConstants.ARM_PIVOT_X_OFFSET;
        angleForArm = Math.toDegrees(Math.atan(speakerHeight / distanceFromRobot)) + MainConstants.ARM_ORIGINAL_DEGREES;

        return angleForArm;
    }

    public double armSpeakersAligningBlue() {
        double angleForArm;
        double distanceFromRobot;
        distanceFromRobot = drive.getPose().getTranslation().getDistance(new Translation2d(0.2, 5.547));

        // double distanceAimSpeaker = (drive.getPose().getTranslation().getDistance(new Translation2d(0.01, 5.547)) - 1.27) * (0.700 - 0.645) / (5.7912 - 1.27) + 0.645;
        double speakerHeight = ((MainConstants.SPEAKER_Z) - MainConstants.ARM_PIVOT_Z) -.045;
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

    public CANcoder getArmEncoder() {
        if (!subsystemStatus) return null;
        return armCANCoder;
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

    public Command goToQueuedSetpoint(){
        return this.runOnce(()->setTrapezoidalProfileSetpoint(setPointInQue));
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
        return this.runOnce(() -> setPointInQue =  MainConstants.Setpoints.ARM_AMP_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Back setpoint
     */
    public Command rotateBack() {
        return this.runOnce(() -> setPointInQue =  MainConstants.Setpoints.ARM_SPEAKER_BACK_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Front setpoint
     */
    public Command rotateFront() {
        return this.runOnce(() -> setPointInQue =  MainConstants.Setpoints.ARM_SPEAKER_FRONT_SETPOINT);
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
        return this.runOnce(() -> setTrapezoidalProfileSetpoint(MainConstants.Setpoints.ARM_TRAP_PREP_SETPOINT));
    }

    /**
     * Sets the Arm setpoint to the Arm Subwoofer setpoint
     */
    public Command rotateSub() {
        return this.runOnce(() -> setPointInQue = MainConstants.Setpoints.ARM_SUBWOOFER_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Podium setpoint
     */
    public Command rotateSafe() {
        return this.runOnce(() -> setPointInQue = MainConstants.Setpoints.ARM_SAFE_SETPOINT);
    }

    public Command rotateFarShot() {
        return this.runOnce(() -> setPointInQue =  MainConstants.Setpoints.ARM_FAR_SHOT_SETPOINT);
    }

    public Command rotateHPStation() {
        return this.runOnce(() -> setPointInQue = MainConstants.Setpoints.ARM_HP_STATION_SETPOINT);
    }
    
    public Command rotateAutonStable() {
        return this.runOnce(() -> setTrapezoidalProfileSetpoint(MainConstants.Setpoints.ARM_AUTON_STABLE));
    }

    private void setTrapezoidalProfileSetpoint(double setpoint) {
        // todo
        
        rotateSetpoint= setpoint;
        // trap constraints degrees
        // rotateSetpoint  degrees?
        //
        if(climbMode) {
        // trapProfile = new TrapezoidProfile(climbTrapConstraints, new TrapezoidProfile.State(rotateSetpoint,0), new TrapezoidProfile.State(encoderValue, armEncoder.getVelocity().getValue()*360.0));

        } else  {
        if((rotateSetpoint > 120 & encoderValue > 120) || (setpoint < 120 &&  encoderValue < 120)){
            //    trapProfile = new TrapezoidProfile(trapConstraints, new TrapezoidProfile.State(rotateSetpoint,0), new TrapezoidProfile.State(encoderValue, (armEncoder.getVelocity().getValue()/80)*360));
        }
        else{
            //     trapProfile = new TrapezoidProfile(crossTrapContrainrs, new TrapezoidProfile.State(rotateSetpoint, 0), new TrapezoidProfile.State(encoderValue, (armEncoder.getVelocity().getValue()/80)*360));
        }
    }
        
        timer.restart();   
        //prevState = trapProfile.calculate(timer.get());

    }

}