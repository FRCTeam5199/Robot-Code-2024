package frc.robot.subsystems;

import java.rmi.server.ExportException;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;


public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem armSubsystem;

    private static boolean subsystemStatus = false;

    private static VortexMotorController armMotorL;
    private static VortexMotorController armMotorR;
    private static boolean isBrakeMode = false;
    public boolean inAuton = false;
    public double encoderValue;
    private CANSparkMax armEncoderMotor;
    private SparkAbsoluteEncoder armEncoder;
    private PIDController rotatePIDController;
    public  boolean isAiming = false;
    public boolean autoAiming = false;

    private double rotateSetpoint = 120;
    private double rotateOffset;

    SwerveDrive drive = TunerConstants.DriveTrain;

    public ArmSubsystem() {}

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
        armMotorL.setBrake(isBrakeMode);
		armMotorR.setBrake(isBrakeMode);
    }

    /**
     * init for arm and pid controller
     */
    public void init() {
        motorInit();
        //  try { motorInit(); } catch (Exception exception) {
        // 	System.err.println("One or more issues occured while trying to initalize motors for Arm Subsystem");
        // 	System.err.println("Exception Message:" + exception.getMessage());
        // 	System.err.println("Exception Cause:" + exception.getCause());
        // 	System.err.println("Exception Stack Trace:" + exception.getStackTrace()); }

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
        armMotorL = new VortexMotorController(MainConstants.IDs.Motors.ARM_MOTOR_ID);
        armMotorR = new VortexMotorController(MainConstants.IDs.Motors.ARM_MOTOR2_ID);

        armMotorL.setInvert(false);
        armMotorL.setBrake(true);

        armMotorR.setInvert(true);
        armMotorR.setBrake(true);

        armMotorL.setCurrentLimit(40);
        armMotorR.setCurrentLimit(40);

        armEncoderMotor = new CANSparkMax(MainConstants.IDs.Motors.ARM_ENCODER_MOTOR, MotorType.kBrushed);
        armEncoder = armEncoderMotor.getAbsoluteEncoder(Type.kDutyCycle);
        
    }

    public void PIDInit() {
        
        rotatePIDController = new PIDController(0.0083262, 0.00569673212, 0.003);
        rotatePIDController.setIZone(3);
        
        rotatePIDController.setTolerance(encoderValue);
    }

    public boolean checkMotors() {
        if (armMotorL != null && armEncoderMotor != null && armEncoder != null) {
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
        if (checkMotors() && checkPID()) { subsystemStatus = true; } else { subsystemStatus = false; }
        
        if(subsystemStatus){
            subsystemPeriodic();
        }
    }

    private void subsystemPeriodic() {
        if (armEncoder.getPosition() < 170) {
            encoderValue = armEncoder.getPosition();
        } else if (armEncoder.getPosition() >170 && armEncoder.getPosition() < 200){
            encoderValue = 170;
        } else if (armEncoder.getPosition() >200 && armEncoder.getPosition() < 361){
            encoderValue = 0;
        } else if (armEncoder.getPosition() > 200){
             armMotorL.set(rotatePIDController.calculate(encoderValue, 0));
            armMotorR.set(rotatePIDController.calculate(encoderValue, 0));
        }
        if (inAuton) {
            goToSetpoint(rotateSetpoint, rotateOffset);
        }
            if(isAiming){   
                System.out.println("encoder value " + encoderValue);
                System.out.println("rotate setPoint " + rotateSetpoint);
                goToSetpoint(rotateSetpoint, rotateOffset);
            } else {
                goToSetpoint(MainConstants.Setpoints.ARM_STABLE_SETPOINT, rotateOffset);
            }
        }
    private void goToSetpoint(double rotateSetpoint, double rotateOffset){
        // System.out.println();
        armMotorL.set(rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset));
        armMotorR.set(rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset));
    
      
    }

    public Command isAutoAiming(boolean bool){
        return this.runOnce(()-> autoAiming = bool);
    }

    public Command isAiming(boolean bool){
        return this.runOnce(()-> isAiming = bool);
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



    public Command autoAlignSpeaker(double setPoint){
        return new SequentialCommandGroup(new InstantCommand(()-> isAiming = false),new InstantCommand(()-> goToSetpoint(setPoint, 0)));
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
        return this.runOnce(()-> System.out.println("changed to " + setpoint)).andThen(() -> rotateSetpoint = setpoint);
    }

    public void setAutoAimingSetpoint(double setpoint){
        System.out.println("changed to " + setpoint);
        rotateSetpoint = setpoint;
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
    public Command rotateClimb() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_CLIMB_SETPOINT);
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
    public Command rotateTrap() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TRAP_SETPOINT);
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